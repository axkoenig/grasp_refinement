import gym
import numpy as np

import rospy
from std_msgs.msg import Float32
from reflex_msgs.msg import PoseCommand, Hand

from .helpers import rad2deg, deg2rad
from .space import Space, Variable
from .gazebo_interface import GazeboInterface

# TODO clip observation_space if needed and alert (check if Gym does this by default)
# TODO experiment stopping CONDITIONS end if object too further than max wrist obj distance away
# TODO check what seed does


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.prox_angle = Variable("prox_angle", 0, 0, 3)
        self.dist_angle = Variable("dist_angle", 0, 0, 0.2 * self.prox_angle.max_val)  # as defined in finger_controller.cpp
        self.contact_pressure = Variable("contact_pressure", 0, 0, 5)
        self.wrist_obj_dist = Variable("wrist_obj_dist", 0, 0, 0.2)

        self.add_variable(self.prox_angle, 3)
        self.add_variable(self.dist_angle, 3)
        self.add_variable(self.contact_pressure, 27)
        self.add_variable(self.wrist_obj_dist, 3)


class ActionSpace(Space):
    """Defines action space"""

    def __init__(self):
        super().__init__()

        self.finger_incr = Variable("finger_incr", 0, deg2rad(-10), deg2rad(10))
        self.wrist_z_incr = Variable("dist_angle", 0, -0.03, 0.03)

        self.add_variable(self.finger_incr, 3)
        self.add_variable(self.wrist_z_incr, 1)


class GazeboEnv(gym.Env):
    def __init__(self, exec_secs, max_ep_len, joint_lim):

        self.exec_secs = exec_secs
        self.max_ep_len = max_ep_len
        self.joint_lim = joint_lim
        self.epsilon_scaling = 10

        rospy.init_node("agent", anonymous=True)

        self.gazebo_interface = GazeboInterface()
        self.hand_cmd = PoseCommand()
        self.actions = ActionSpace()
        self.observations = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.actions.get_min_vals(), high=self.actions.get_max_vals(), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=self.observations.get_min_vals(), high=self.observations.get_max_vals(), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

        self.hand_pub = rospy.Publisher("reflex/pos_cmd", PoseCommand, queue_size=5)
        self.reward_pub = rospy.Publisher("agent/reward", Float32, queue_size=5)
        self.hand_sub = rospy.Subscriber("reflex/hand_state", Hand, self.hand_callback, queue_size=5)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_callback(self, msg):
        self.observations.vars[0].cur_val = msg.finger[0].proximal
        self.observations.vars[1].cur_val = msg.finger[1].proximal
        self.observations.vars[2].cur_val = msg.finger[2].proximal

    def step(self, action):
        self.hand_cmd.f1 = action[0]
        self.hand_cmd.f2 = action[1]
        self.hand_cmd.f3 = action[2]

        # TODO add more actions here
        self.hand_pub.publish(self.hand_cmd)
        cmd_str = f"{self.hand_cmd.f1}, {self.hand_cmd.f2}, {self.hand_cmd.f3}"
        self.gazebo_interface.run_for_seconds("Step", self.exec_secs, cmd_str)

        des_angle = 1
        reward = 0
        done = False
        logs = {}

        # TODO update reward
        reward_msg = Float32(reward)
        self.reward_pub.publish(reward_msg)

        prox_angles = self.observations.vars[:3]
        if not all(prox_angle.cur_val < self.joint_lim for prox_angle in prox_angles):
            done = True
            rospy.loginfo(f"One angle is above {self.joint_lim} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.max_ep_len} secs. Setting done = True.")

        return self.observations.get_cur_vals(), reward, done, logs

    def reset(self):
        # opening hand
        self.hand_cmd.f1 = 0
        self.hand_cmd.f2 = 0
        self.hand_cmd.f3 = 0
        self.hand_pub.publish(self.hand_cmd)
        cmd_str = f"{self.hand_cmd.f1}, {self.hand_cmd.f2}, {self.hand_cmd.f3}"
        self.gazebo_interface.run_for_seconds("Reset", 1, cmd_str)

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")
