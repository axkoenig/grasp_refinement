import gym
import numpy as np

import rospy
from std_msgs.msg import Float64, Int32
from reflex_msgs.msg import PoseCommand, Hand
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
        self.num_contact_pressures = self.num_fingers * self.num_sensors
        self.num_wrist_obj_dists = 3
        self.prox_angle_max = 3

        self.add_variable(self.num_motors, "prox_angle", 0, 0, self.prox_angle_max)
        self.add_variable(self.num_fingers, "dist_angle", 0, 0, 0.2 * self.prox_angle_max)
        self.add_variable(self.num_contact_pressures, "contact_pressure", 0, 0, 10)


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.add_variable(1, "trigger_regrasp", 0, 0, 1)
        self.add_variable(3, "wrist_incr", 0, -0.01, 0.01)


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
        self.cur_num_contacts = self.training_env.get_attr("num_contacts")[0]
        self.cur_dist_tcp_obj = self.training_env.get_attr("dist_tcp_obj")[0]
        self.cur_epsilon_force = self.training_env.get_attr("epsilon_force")[0]
        self.cur_epsilon_torque = self.training_env.get_attr("epsilon_torque")[0]
        self.cur_obj_shift = self.training_env.get_attr("obj_shift")[0]

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
    def __init__(self, exec_secs, max_ep_len, joint_lim, obj_shift_tol):

        self.exec_secs = exec_secs
        self.max_ep_len = max_ep_len
        self.joint_lim = joint_lim
        self.obj_shift_tol = obj_shift_tol

        rospy.init_node("agent", anonymous=True)

        self.gazebo_interface = GazeboInterface()

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
        self.hand_sub = rospy.Subscriber("reflex/hand_state", Hand, self.hand_callback, queue_size=5)
        self.num_contacts_sub = rospy.Subscriber("reflex/num_contacts", Int32, self.num_contacts_callback, queue_size=5)
        self.epsilon_force_sub = rospy.Subscriber("reflex/epsilon_force", Float64, self.epsilon_force_callback, queue_size=5)
        self.epsilon_torque_sub = rospy.Subscriber("reflex/epsilon_torque", Float64, self.epsilon_torque_callback, queue_size=5)

        self.num_contacts = 0
        self.epsilon_force = 0
        self.epsilon_torque = 0

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def num_contacts_callback(self, msg):
        self.num_contacts = msg.data

    def epsilon_force_callback(self, msg):
        self.epsilon_force = msg.data

    def epsilon_torque_callback(self, msg):
        self.epsilon_torque = msg.data

    def hand_callback(self, msg):

        for i in range(self.obs.num_fingers):
            # joint pos 1 to 3 from magnetic encoders (more accurate than motor encoders)
            self.obs.vars[i].cur_val = msg.finger[i].proximal
            self.obs.vars[i + self.obs.num_motors].cur_val = msg.finger[i].distal_approx

            for j in range(self.obs.num_sensors):
                self.obs.vars[i + self.obs.num_motors + self.obs.num_fingers + j].cur_val = msg.finger[i].pressure[j]

        # joint pos 4 from motor encoder (there is no magnetic encoder)
        self.obs.vars[3].cur_val = msg.motor[3].joint_angle

    def step(self, action):

        if action[0] > 0.5:
            self.gazebo_interface.regrasp([action[1], action[2], action[3]])
        else:
            # run for a short time, and collect new reward
            self.gazebo_interface.run_for_seconds(0.01)

        # get object shift and distance to object (used for logging)
        t_obj, _ = get_tq_from_homo_matrix(self.gazebo_interface.get_object_pose())
        self.obj_shift = np.linalg.norm(t_obj - self.t_obj_init)
        self.dist_tcp_obj = self.gazebo_interface.get_dist_tcp_obj()

        reward = self.epsilon_force + self.epsilon_torque
        reward_msg = Float64(reward)
        self.reward_pub.publish(reward_msg)

        done = False
        logs = {}

        # check if should end episode
        if self.obj_shift > self.obj_shift_tol:
            done = True
            rospy.loginfo(f"Object shift is above {self.obj_shift_tol} m. Setting done = True.")
        elif not all(prox_angle.cur_val < self.joint_lim for prox_angle in self.obs.vars[:3]):
            done = True
            rospy.loginfo(f"One angle is above {self.joint_lim} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.max_ep_len} secs. Setting done = True.")

        return self.obs.get_cur_vals(True), reward, done, logs

    def reset(self):
        self.gazebo_interface.reset_world(self.wrist_init_pose, self.obj_init_pose)

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")
