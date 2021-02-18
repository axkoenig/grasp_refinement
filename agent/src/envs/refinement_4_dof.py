import gym
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from reflex_msgs.msg import PoseCommand, Hand

from .helpers import rad2deg, deg2rad

# TODO clip observation_space if needed and alert (check if Gym does this by default)
# TODO experiment stopping CONDITIONS end if object too further than max wrist obj distance away
# TODO check what seed does

class Variable:
    """Defines one action or observation variable."""

    def __init__(self, name, init_value, min_val, max_val):
        self.name = name
        self.cur_val = init_value
        self.min_val = min_val
        self.max_val = max_val


class Space:
    """Parent class for observation and action class."""
    def __init__(self):
        self.vars = []
        self.dim = 0

    def get_space_low(self):
        space_low = np.empty((0,))
        for i in range(self.dim):
            space_low = np.append(space_low, self.vars[i].min_val)
        print("space_low of " + self.__class__.__name__ + f" is {space_low}.")
        return space_low

    def get_space_high(self):
        space_high = np.empty((0,))
        for i in range(self.dim):
            space_high = np.append(space_high, self.vars[i].max_val)

        print("space_high of " + self.__class__.__name__ + f" is {space_high}.")
        return space_high

    def add_variable(self, var, num_instances):
        self.vars.extend([var for i in range(num_instances)])
        self.dim = len(self.vars)


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.prox_angle = Variable("prox_angle", 0, 0, 3)
        self.dist_angle = Variable("dist_angle", 0, 0, 0.2 * self.prox_angle.max_val)  # as defined in finger_controller.cpp
        self.contact_pressure = Variable("contact_pressure", 0, 0, 5)
        self.wrist_obj_dist = Variable("wrist_obj_dist", 0, 0, 0.2)

        print(self.prox_angle.max_val)
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

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)

        self.hand_cmd = PoseCommand()
        self.actions = ActionSpace()
        self.observations = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.actions.get_space_low(), high=self.actions.get_space_high(), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=self.observations.get_space_low(), high=self.observations.get_space_high(), dtype=np.float32)
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

    def sim_unpause(self):
        rospy.wait_for_service(self.unpause_name)
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.unpause_name + " service call failed")

    def sim_pause(self):
        rospy.wait_for_service(self.pause_name)
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.pause_name + " service call failed")

    def execute_for_seconds(self, secs, prefix):
        self.hand_pub.publish(self.hand_cmd)
        self.sim_unpause()
        rospy.loginfo(f"{prefix}: Executing {self.hand_cmd.f1}, {self.hand_cmd.f2}, {self.hand_cmd.f3} for {secs} secs.")
        rospy.sleep(secs)
        self.sim_pause()

    def step(self, action):
        self.hand_cmd.f1 = action[0]
        self.hand_cmd.f2 = action[1]
        self.hand_cmd.f3 = action[2]

        # TODO add more actions here 
        self.execute_for_seconds(self.exec_secs, "Step")

        # obtain observations
        obs = np.zeros(self.observation_space.shape)
        for i in range(self.observations.dim): 
            obs[i] = self.observations.vars[i].cur_val

        des_angle = 1
        reward = 0
        done = False
        logs = {}

        # TODO update reward
        reward_msg = Float32(reward)
        self.reward_pub.publish(reward_msg)
        
        if obs[0] > self.joint_lim or obs[1] > self.joint_lim or obs[2] > self.joint_lim:
            done = True
            rospy.loginfo(f"One angle is above {self.joint_lim} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.max_ep_len} secs. Setting done = True.")

        return obs, reward, done, logs

    def reset(self):
        # opening hand
        self.hand_cmd.f1 = 0
        self.hand_cmd.f2 = 0
        self.hand_cmd.f3 = 0
        self.execute_for_seconds(1, "Reset")

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")
