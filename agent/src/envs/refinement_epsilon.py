import gym
import numpy as np

import rospy
from std_msgs.msg import Float64, Int32
from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import PoseCommand
from stable_baselines3.common.callbacks import BaseCallback

from .helpers import rad2deg, deg2rad, get_homo_matrix_from_tq, get_tq_from_homo_matrix
from .space import Space
from .gazebo_interface import GazeboInterface


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.num_fingers = 3
        self.num_motors = 4
        self.num_sensors = 9
        self.num_tot_sensors = self.num_fingers * self.num_sensors
        self.vars_per_finger = 4 + 2 * self.num_motors
        self.prox_angle_max = 3

        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable(1, "prox_angle" + id_str, 0, 0, self.prox_angle_max)
            self.add_variable(1, "dist_angle" + id_str, 0, 0, 0.2 * self.prox_angle_max)
            self.add_variable(1, "prox_normal" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])
            self.add_variable(1, "dist_normal" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])

            for j in range(self.num_sensors):
                id_str = "_f" + str(i + 1) + "_s" + str(j + 1)
                self.add_variable(1, "sensor_pressure" + id_str, 0, 0, 10)
                self.add_variable(1, "tactile_positions" + id_str, [0, 0, 0], [-0.2, -0.16, 0.06], [0.2, 0.16, 0.2])

        self.add_variable(1, "preshape_angle", 0, 0, self.prox_angle_max)


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.add_variable(1, "trigger_regrasp", 0, 0, 1)
        self.add_variable(2, "wrist_incr", 0, -0.001, 0.001)
        self.add_variable(1, "wrist_incr_z", 0, -0.005, 0.02)

        # self.add_variable(3, "finger_incr", 0, -0.01, 0.1)
        # self.add_variable(2, "wrist_incr", 0, -0.0001, 0.0001)
        # self.add_variable(1, "wrist_incr_z", 0, -0.0005, 0.003)


class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0

    def _on_rollout_end(self) -> None:
        self.logger.record("rollout/cum_num_contacts", self.cum_num_contacts)
        self.logger.record("rollout/cum_dist_tcp_obj", self.cum_dist_tcp_obj)
        self.logger.record("rollout/cum_epsilon_force", self.cum_epsilon_force)
        self.logger.record("rollout/cum_epsilon_torque", self.cum_epsilon_torque)
        self.logger.record("rollout/cum_obj_shift", self.cum_obj_shift)

        # reset vars once recorded
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0

    def _on_step(self) -> bool:
        self.cur_num_regrasps = self.training_env.get_attr("num_regrasps")[0]
        self.cur_num_contacts = self.training_env.get_attr("num_contacts")[0]
        self.cur_dist_tcp_obj = self.training_env.get_attr("dist_tcp_obj")[0]
        self.cur_epsilon_force = self.training_env.get_attr("epsilon_force")[0]
        self.cur_epsilon_torque = self.training_env.get_attr("epsilon_torque")[0]
        self.cur_obj_shift = self.training_env.get_attr("obj_shift")[0]

        self.logger.record("step/cur_num_regrasps", self.cur_num_regrasps)
        self.logger.record("step/cur_num_contacts", self.cur_num_contacts)
        self.logger.record("step/cur_dist_tcp_obj", self.cur_dist_tcp_obj)
        self.logger.record("step/cur_epsilon_force", self.cur_epsilon_force)
        self.logger.record("step/cur_epsilon_torque", self.cur_epsilon_torque)
        self.logger.record("step/cur_obj_shift", self.cur_obj_shift)

        self.cum_num_contacts += self.cur_num_contacts
        self.cum_dist_tcp_obj += self.cur_dist_tcp_obj
        self.cum_epsilon_force += self.cur_epsilon_torque
        self.cum_epsilon_torque += self.cur_epsilon_force
        self.cum_obj_shift += self.cur_obj_shift
        return True


class GazeboEnv(gym.Env):
    def __init__(self, exec_secs, max_ep_len, joint_lim, obj_shift_tol, reward_weight):

        self.exec_secs = exec_secs
        self.max_ep_len = max_ep_len
        self.joint_lim = joint_lim
        self.obj_shift_tol = obj_shift_tol
        self.reward_weight = reward_weight

        rospy.init_node("agent", anonymous=True)

        self.gi = GazeboInterface()

        self.wrist_init_pose = get_homo_matrix_from_tq(
            [-0.020000008416459117, 0.037719972837563544, 0.03613883173559584],
            [-0.7075026182105896, 0, 0, 0.7067107101727882],
        )
        self.t_obj_init = [1.2079885446707724e-05, 0.2000036106758952, 0.02999998861373354]
        self.obj_init_pose = get_homo_matrix_from_tq(
            self.t_obj_init,
            [-2.0374825955119477e-07, 3.561732066170289e-08, 0.0002924163549034363, 0.9999999572463154],
        )

        self.hand_cmd = PoseCommand()
        self.acts = ActionSpace()
        self.obs = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals(), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals(), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

        self.hand_pub = rospy.Publisher("reflex/pos_cmd", PoseCommand, queue_size=5)
        self.reward_pub = rospy.Publisher("agent/reward", Float64, queue_size=5)
        self.hand_state_sub = rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.hand_state_callback, queue_size=5)

        self.num_contacts = 0
        self.epsilon_force = 0
        self.epsilon_torque = 0
        self.num_regrasps = 0
        self.last_quality = 0

    def seed(self, seed=None):
        import pdb; pdb.set_trace()
        
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_state_callback(self, msg):
        self.num_contacts = msg.num_contacts
        self.epsilon_force = msg.epsilon_force
        self.epsilon_torque = msg.epsilon_torque

        self.obs.set_cur_val_by_name("preshape_angle", msg.preshape_angle)

        for i in range(self.obs.num_fingers):
            # joint positions
            id_str = "_f" + str(i + 1)
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
                    tactile_positions = self.gi.ros_vector_to_list(msg.finger_state[i].tactile_position[j])
                    self.obs.set_cur_val_by_name("tactile_positions" + id_str, tactile_positions)
                else:
                    self.obs.set_cur_val_by_name("tactile_positions" + id_str, [0, 0, 0])

    def collect_reward(self, tot_duration=0.5, time_steps=20):
        # records epsilon over tot_duration and returns average
        sleep_time = tot_duration / time_steps
        reward = 0
        self.gi.sim_unpause()
        for i in range(time_steps):
            # multiplying by 10 because size of torque epsilon is approx 1/10 of force epsilon
            reward += self.get_reward()
            rospy.sleep(sleep_time)
        self.gi.sim_pause()
        return reward / time_steps
    
    def get_reward(self):
        return self.epsilon_force + 10 * self.epsilon_torque

    def step(self, action):
        #### REGRASPING
        rospy.loginfo(f"Action is {action[0]}")

        if 0.5 <= action[0]:
            rospy.loginfo("Regrasping.")
            self.gi.regrasp([action[1], action[2], action[3]])
            self.num_regrasps += 1
        else:
            rospy.loginfo("Staying.")
        ### REGRASPING END


        prox_angles = [
            self.obs.get_cur_vals_by_name("prox_angle_f1"),
            self.obs.get_cur_vals_by_name("prox_angle_f2"),
            self.obs.get_cur_vals_by_name("prox_angle_f3"),
        ]

        # # send finger position increment
        # self.hand_cmd.f1 = prox_angles[0] + action[0]
        # self.hand_cmd.f2 = prox_angles[1] + action[1]
        # self.hand_cmd.f3 = prox_angles[2] + action[2]
        # self.hand_pub.publish(self.hand_cmd)

        # # send wrist position increment
        # self.gi.cmd_wrist_pos_incr([action[3], action[4], action[5]])

        # rospy.loginfo(f"Actions were {action[0]}, {action[1]}, {action[2]}, {action[3]}, {action[4]}, {action[5]}")
        cur_reward = self.collect_reward(self.exec_secs)
        reward = cur_reward - self.last_reward
        rospy.loginfo(f"Relative reward is {reward}")
        reward_msg = Float64(reward)
        self.reward_pub.publish(reward_msg)

        # get object shift and distance to object (used for logging)
        t_obj, _ = get_tq_from_homo_matrix(self.gi.get_object_pose())
        self.obj_shift = np.linalg.norm(t_obj - self.t_obj_init)
        self.dist_tcp_obj = self.gi.get_dist_tcp_obj()

        done = False
        logs = {}

        # check if should end episode
        if self.obj_shift > self.obj_shift_tol:
            done = True
            rospy.loginfo(f"Object shift is above {self.obj_shift_tol} m. Setting done = True.")
        elif not all(prox_angle < self.joint_lim for prox_angle in prox_angles):
            done = True
            rospy.loginfo(f"One angle is above {self.joint_lim} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.max_ep_len} secs. Setting done = True.")

        return self.obs.get_cur_vals(), reward, done, logs

    def reset(self):
        rospy.loginfo("Resetting world.")
        self.gi.reset_world(self.wrist_init_pose, self.obj_init_pose)
        self.last_reward = self.collect_reward(self.exec_secs)

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        self.num_regrasps = 0
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
