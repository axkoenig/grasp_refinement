import gym
import numpy as np

import rospy
import tf
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
        self.add_variable(1, "wrist_incr_x", 0, -0.02, 0.02)
        self.add_variable(1, "wrist_incr_y", 0, -0.001, 0.001)
        self.add_variable(1, "wrist_incr_z", 0, -0.005, 0.02)
        self.add_variable(1, "wrist_pitch", 0, -0.1, 0.1)
        self.add_variable(3, "trigger_finger_tightening", 0, 0, 1)


class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0
        self.cum_joint_diff = 0
        self.cum_delta_task = 0

    def _on_rollout_end(self) -> None:
        self.logger.record("rollout/cum_num_contacts", self.cum_num_contacts)
        self.logger.record("rollout/cum_dist_tcp_obj", self.cum_dist_tcp_obj)
        self.logger.record("rollout/cum_epsilon_force", self.cum_epsilon_force)
        self.logger.record("rollout/cum_epsilon_torque", self.cum_epsilon_torque)
        self.logger.record("rollout/cum_obj_shift", self.cum_obj_shift)
        self.logger.record("rollout/cum_joint_diff", self.cum_joint_diff)
        self.logger.record("rollout/cum_delta_task", self.cum_delta_task)

        # reset vars once recorded
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0
        self.cum_joint_diff = 0
        self.cum_delta_task = 0

    def _on_step(self) -> bool:
        self.cur_num_regrasps = self.training_env.get_attr("num_regrasps")[0]
        self.cur_num_contacts = self.training_env.get_attr("num_contacts")[0]
        self.cur_dist_tcp_obj = self.training_env.get_attr("dist_tcp_obj")[0]
        self.cur_epsilon_force = self.training_env.get_attr("epsilon_force")[0]
        self.cur_epsilon_torque = self.training_env.get_attr("epsilon_torque")[0]
        self.cur_delta_task = self.training_env.get_attr("delta_task")[0]
        self.cur_obj_shift = self.training_env.get_attr("obj_shift")[0]
        self.cur_joint_diff = self.training_env.get_attr("prox_diff")[0]

        self.logger.record("step/cur_num_regrasps", self.cur_num_regrasps)
        self.logger.record("step/cur_num_contacts", self.cur_num_contacts)
        self.logger.record("step/cur_dist_tcp_obj", self.cur_dist_tcp_obj)
        self.logger.record("step/cur_epsilon_force", self.cur_epsilon_force)
        self.logger.record("step/cur_epsilon_torque", self.cur_epsilon_torque)
        self.logger.record("step/cur_delta_task", self.cur_delta_task)
        self.logger.record("step/cur_obj_shift", self.cur_obj_shift)
        self.logger.record("step/cur_joint_diff", self.cur_joint_diff)

        self.cum_num_contacts += self.cur_num_contacts
        self.cum_dist_tcp_obj += self.cur_dist_tcp_obj
        self.cum_epsilon_force += self.cur_epsilon_torque
        self.cum_epsilon_torque += self.cur_epsilon_force
        self.cum_obj_shift += self.cur_obj_shift
        self.cum_joint_diff += self.cur_joint_diff
        self.cum_delta_task += self.cur_delta_task
        return True


class GazeboEnv(gym.Env):
    def __init__(self, exec_secs, max_ep_len, joint_lim, obj_shift_tol, reward_weight, pos_error):

        self.exec_secs = exec_secs
        self.max_ep_len = max_ep_len
        self.joint_lim = joint_lim
        self.obj_shift_tol = obj_shift_tol
        self.reward_weight = reward_weight
        self.x_error = pos_error[0]
        self.y_error = pos_error[1]
        self.z_error = pos_error[2]

        rospy.init_node("agent", anonymous=True)

        self.gi = GazeboInterface()

        self.wrist_init_pose = get_homo_matrix_from_tq(
            [-0.020000008416459117, 0.027719972837563544, 0.03613883173559584],
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
        self.delta_task = 0
        self.num_regrasps = 0
        self.last_quality = 0
        self.cur_time_step = 0

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_state_callback(self, msg):
        self.num_contacts = msg.num_contacts
        self.epsilon_force = msg.epsilon_force
        self.epsilon_torque = msg.epsilon_torque
        self.delta_task = msg.delta_task

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
            reward += self.get_reward()
            rospy.sleep(sleep_time)
        self.gi.sim_pause()
        return reward / time_steps

    def get_reward(self):
        return self.epsilon_force + 10 * self.epsilon_torque + 1 / 100 * self.delta_task

    def get_f_incr(self, action):
        return 0.1 if action >= 0.5 else 0

    def step(self, action):
        self.cur_time_step += 1

        rospy.loginfo(f"Action Regrasp \t {action[0]}")
        rospy.loginfo(f"Action Wrist \t {action[1]}, {action[2]}, {action[3]}, {action[4]}")
        rospy.loginfo(f"Action Finger \t {action[5]}, {action[6]}, {action[7]}")

        if action[0] >= 0:
            rospy.loginfo(">>REGRASPING<<")
            wrist_p_incr = [action[1], action[2], action[3]]
            wrist_q_incr = tf.transformations.quaternion_from_euler(0, action[4], 0)
            self.gi.regrasp(wrist_p_incr, wrist_q_incr)
            self.num_regrasps += 1
        else:
            rospy.loginfo(">>STAYING<<")
            self.gi.pos_incr(get_f_incr(action[5]), get_f_incr(action[6]), get_f_incr(action[7]), 0, False, False, 0, 0)

        #### STUFF ABOUT PROX ANGLES
        prox_angles = [
            self.obs.get_cur_vals_by_name("prox_angle_f1"),
            self.obs.get_cur_vals_by_name("prox_angle_f2"),
            self.obs.get_cur_vals_by_name("prox_angle_f3"),
        ]

        self.prox_diff = abs(prox_angles[0] - prox_angles[1]) + abs(prox_angles[1] - prox_angles[2]) + abs(prox_angles[0] - prox_angles[2])
        self.prox_diff = self.prox_diff[0]  # convert to float
        rel_prox_diff_change = self.prox_diff - self.start_prox_diff
        #### END STUFF ABOUT PROX ANGLES

        # reward is relative grasp improvement w.r.t. starting config
        reward = self.collect_reward(self.exec_secs) - self.start_reward
        rospy.loginfo(f"==> reward is {reward}")

        reward_msg = Float64(reward)
        self.reward_pub.publish(reward_msg)

        # get object shift and distance to object
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
        elif self.cur_time_step == self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.cur_time_step} time steps. Setting done = True.")

        return self.obs.get_cur_vals(), reward, done, logs

    def reset(self):
        # generate random offset from initial wrist pose
        x_offset = np.random.uniform(-self.x_error, self.x_error)
        y_offset = np.random.uniform(-self.y_error, self.y_error)
        z_offset = np.random.uniform(-self.z_error, self.z_error)
        rospy.loginfo(f"Random offset for init wrist pose is [x: {x_offset}, y: {y_offset}, z: {z_offset}].")
        mat_offset = tf.transformations.translation_matrix([x_offset, y_offset, z_offset])
        wrist_init_pose_err = np.dot(self.wrist_init_pose, mat_offset)

        rospy.loginfo("Resetting world.")
        self.gi.reset_world(wrist_init_pose_err, self.obj_init_pose)
        self.start_reward = self.collect_reward(self.exec_secs)

        prox_angles = [
            self.obs.get_cur_vals_by_name("prox_angle_f1"),
            self.obs.get_cur_vals_by_name("prox_angle_f2"),
            self.obs.get_cur_vals_by_name("prox_angle_f3"),
        ]
        self.start_prox_diff = abs(prox_angles[0] - prox_angles[1]) + abs(prox_angles[1] - prox_angles[2]) + abs(prox_angles[0] - prox_angles[2])
        self.start_prox_diff = self.start_prox_diff[0]
        rospy.loginfo(f"start_reward is {self.start_reward}")
        rospy.loginfo(f"start_prox_diff is {self.start_prox_diff}")

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        self.num_regrasps = 0
        self.cur_time_step = 0
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
