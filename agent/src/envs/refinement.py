import gym
import numpy as np
import rospy
import tf

from std_msgs.msg import Float64, Int32
from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import PoseCommand

from .helpers import rad2deg, deg2rad, get_homo_matrix_from_tq, get_tq_from_homo_matrix
from .space import Space
from .space_act import ActionSpace
from .space_obs import ObservationSpace
from .gazebo_interface import GazeboInterface


class GazeboEnv(gym.Env):
    def __init__(self, hparams):

        self.hparams = hparams

        # quality metrics
        self.epsilon_force = 0
        self.epsilon_torque = 0
        self.delta_task = 0
        self.delta_cur = 0

        # counters
        self.num_regrasps = 0
        self.cur_time_step = 0

        rospy.init_node("agent", anonymous=True)

        self.gi = GazeboInterface()
        self.acts = ActionSpace()
        self.obs = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals())
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals())
        self.reward_range = (-np.inf, np.inf)

        self.hand_state_sub = rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.hand_state_callback, queue_size=5)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_state_callback(self, msg):
        self.num_contacts = msg.num_contacts
        self.epsilon_force = msg.epsilon_force
        self.epsilon_torque = msg.epsilon_torque
        self.delta_task = msg.delta_task
        self.delta_cur = msg.delta_cur
        self.sum_contact_forces = msg.sum_contact_forces
        self.prox_angles = [0,0,0]

        self.obs.set_cur_val_by_name("preshape_angle", msg.preshape_angle)

        for i in range(self.obs.num_fingers):
            # joint positions
            id_str = "_f" + str(i + 1)
            self.prox_angles[i] = msg.finger_state[i].proximal_angle
            self.obs.set_cur_val_by_name("prox_angle" + id_str, msg.finger_state[i].proximal_angle)
            self.obs.set_cur_val_by_name("dist_angle" + id_str, msg.finger_state[i].distal_angle)

            # normals
            normal = self.gi.ros_vector_to_list(msg.finger_state[i].prox_normal)
            self.obs.set_cur_val_by_name("prox_normal" + id_str, normal)
            normal = self.gi.ros_vector_to_list(msg.finger_state[i].dist_normal)
            self.obs.set_cur_val_by_name("dist_normal" + id_str, normal)

            # tactile feedback
            for j in range(self.obs.num_sensors):
                id_str = "_f" + str(i + 1) + "_s" + str(j + 1)
                self.obs.set_cur_val_by_name("sensor_pressure" + id_str, msg.finger_state[i].sensor_pressure[j])

                if msg.finger_state[i].sensor_contact[j]:
                    tactile_position = self.gi.ros_vector_to_list(msg.finger_state[i].tactile_position[j])
                    self.obs.set_cur_val_by_name("tactile_position" + id_str, tactile_position)
                    self.obs.set_cur_val_by_name("tactile_contact" + id_str, 1)

                else:
                    self.obs.set_cur_val_by_name("tactile_position" + id_str, [0, 0, 0])
                    self.obs.set_cur_val_by_name("tactile_contact" + id_str, 0)

    def collect_reward(self, tot_duration=0.5, time_steps=20):
        # records epsilon over tot_duration and returns average
        sleep_time = tot_duration / time_steps
        reward = 0
        self.gi.sim_unpause()
        for i in range(time_steps):
            reward += self.get_reward()
            rospy.sleep(sleep_time)
        self.gi.sim_pause()
        return reward / time_steps

    def get_reward(self):
        return self.epsilon_force + 10 * self.epsilon_torque + 1 / 10 * self.delta_task

    def get_f_incr(self, action):
        return deg2rad(2) if action >= 0.5 else 0

    def step(self, action):
        self.cur_time_step += 1
        rospy.loginfo("===STEP===")
        rospy.loginfo(f"Action Regrasp: \t {action[0]}")
        rospy.loginfo(f"Action Wrist: \t {action[1]}, {action[2]}, {action[3]}, {action[4]}, {action[5]}, {action[6]}")
        rospy.loginfo(f"Action Finger: \t {action[7]}, {action[8]}, {action[9]}")

        if action[0] >= 0.5:
            rospy.loginfo(">>REGRASPING<<")
            wrist_p_incr = [action[1], action[2], action[3]]
            wrist_q_incr = tf.transformations.quaternion_from_euler(action[4], action[5], action[6])
            self.gi.regrasp(wrist_p_incr, wrist_q_incr)
            self.num_regrasps += 1
        else:
            rospy.loginfo(">>STAYING<<")
            self.gi.pos_incr(self.get_f_incr(action[7]), self.get_f_incr(action[8]), self.get_f_incr(action[9]), 0, False, False, 0, 0)

        if self.hparams["framework"] == 1 or self.hparams["framework"] == 3:
            reward = self.collect_reward(self.hparams["exec_secs"])

        logs = {}
        self.done = self.is_done()
        if self.done:
            self.drop_test()

        if self.hparams["framework"] == 2:
            reward = int(self.sustained_holding) if self.done else 0
        elif self.hparams["framework"] == 3:
            binary_reward = int(self.sustained_holding) if self.done else 0
            reward += 2 * binary_reward

        rospy.loginfo(f"--> REWARD: \t {reward}")

        return self.obs.get_cur_vals(), reward, self.done, logs

    def is_done(self):
        # get object shift and distance to object
        obj_t, _ = get_tq_from_homo_matrix(self.gi.get_object_pose())
        self.obj_shift = np.linalg.norm(obj_t - self.gi.start_obj_t)
        self.dist_tcp_obj = self.gi.get_dist_tcp_obj()

        # check if should end episode
        if self.obj_shift > self.hparams["obj_shift_tol"]:
            rospy.loginfo(f"Object shift is above {self.hparams['obj_shift_tol']} m. Setting done = True.")
            return True
        elif not all(prox_angle < self.hparams["joint_lim"] for prox_angle in self.prox_angles):
            rospy.loginfo(f"One angle is above {self.hparams['joint_lim']} rad. Setting done = True.")
            return True
        elif self.cur_time_step == self.hparams["max_ep_len"]:
            rospy.loginfo(f"Episode lasted {self.cur_time_step} time steps. Setting done = True.")
            return True
        return False

    def reset(self):
        rospy.loginfo("===RESETTING===")
        self.gi.reset_world(self.hparams)

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.num_regrasps = 0
        self.cur_time_step = 0
        return obs

    def drop_test(self, lift_vel=0.1, lift_dist=0.15, z_incr=0.0001, secs_to_hold=3):
        counter = 0
        r = rospy.Rate(lift_vel / z_incr)

        # vars to evaluate stability during lifting and when lifted
        self.delta_cur_lifting = 0
        self.delta_cur_holding = 0
        self.eps_force_lifting = 0
        self.eps_force_holding = 0
        self.eps_torque_lifting = 0
        self.eps_torque_holding = 0
        self.sustained_lifting = False
        self.sustained_holding = False

        self.gi.sim_unpause()
        while counter * z_incr <= lift_dist:
            lift_mat = tf.transformations.translation_matrix([0, 0, z_incr])
            lift_mat = np.dot(lift_mat, self.gi.last_wrist_pose)
            self.gi.cmd_wrist_abs(lift_mat)
            r.sleep()
            counter += 1
            self.delta_cur_lifting += self.delta_cur
            self.eps_force_lifting += self.epsilon_force
            self.eps_torque_lifting += self.epsilon_torque

        # average metrics over lifting period
        self.delta_cur_lifting /= counter
        self.eps_force_lifting /= counter
        self.eps_torque_lifting /= counter
        self.sustained_lifting = self.gi.object_lifted()

        if not self.sustained_lifting:
            rospy.loginfo("Object did not sustain lifting.")
            return

        # keep object in hand and record avg stability
        start_time = rospy.Time.now()
        counter = 0
        r = rospy.Rate(100)
        while rospy.Time.now() - start_time <= rospy.Duration(secs_to_hold):
            self.delta_cur_holding += self.delta_cur
            self.eps_force_holding += self.epsilon_force
            self.eps_torque_holding += self.epsilon_torque
            counter += 1
            r.sleep()

        # average metrics over secs_to_hold duration
        self.delta_cur_holding /= counter
        self.eps_force_holding /= counter
        self.eps_torque_holding /= counter

        self.sustained_holding = self.gi.object_lifted()

        self.gi.sim_pause()
