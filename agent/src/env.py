import gym
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from reflex_msgs.msg import PoseCommand, Hand
import rospy


class GazeboEnv(gym.Env):
    def __init__(self):

        rospy.init_node("agent", anonymous=True)

        self.hand_pub = rospy.Publisher("reflex/pos_cmd", PoseCommand, queue_size=5)
        self.reward_pub = rospy.Publisher("agent/reward", Float32, queue_size=5)
        self.hand_sub = rospy.Subscriber(
            "reflex/hand_state", Hand, self.hand_callback, queue_size=5
        )

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)
        self.start_time = rospy.get_rostime()

        # define system state
        self.hand_angles = np.zeros(4)
        self.hand_cmd = PoseCommand()

        # action for one finger (more, stay, less)
        self.action_space = gym.spaces.Discrete(3)
        self.observation_space = gym.spaces.Box(
            low=np.array([0, 0, 0, 0]), high=np.array([3, 3, 3, 3]), dtype=np.float64
        )
        self.reward_range = (-np.inf, np.inf)

        # TODO check what this does
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_callback(self, msg):
        self.hand_angles[0] = msg.finger[0].proximal
        self.hand_angles[1] = msg.finger[1].proximal
        self.hand_angles[2] = msg.finger[2].proximal

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

    def step(self, action):
        self.sim_unpause()

        increment = 0.1

        if action == 0:  # more
            self.hand_cmd.f1 = self.hand_angles[0] + increment
        elif action == 1:  # stay
            self.hand_cmd.f1 = self.hand_angles[0]
        elif action == 2:  # less
            self.hand_cmd.f1 = self.hand_angles[0] - increment

        self.hand_pub.publish(self.hand_cmd)
        rospy.loginfo("Sleeping 0.2 secs.")
        rospy.sleep(0.2)

        self.sim_pause()

        obs = np.zeros(self.observation_space.shape)
        obs[0] = self.hand_angles[0]
        obs[1] = self.hand_angles[1]
        obs[2] = self.hand_angles[2]
        obs[3] = self.hand_angles[3]

        
        des_angle = 1
        reward = 0
        if abs(self.hand_angles[0] - des_angle) < 0.3:
            reward = 1
        reward_msg = Float32(reward)
        self.reward_pub.publish(reward_msg)

        done = False
        logs = {}

        # finish episode when angle above 3 rad
        if obs[0] > 3:
            done = True

        return obs, reward, done, logs

    def reset(self):
        rospy.loginfo("Opening hand again.")
        self.hand_cmd = PoseCommand()
        self.hand_cmd.f1 = 0
        self.hand_pub.publish(self.hand_cmd)

        # make sure sim is unpaused before sleeping
        self.sim_unpause()
        rospy.sleep(3)

        obs = np.zeros(self.observation_space.shape)
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")