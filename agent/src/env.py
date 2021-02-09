import gym
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from reflex_msgs.msg import PoseCommand, Hand


class GazeboEnv(gym.Env):
    def __init__(self):

        rospy.init_node("agent", anonymous=True)

        self.hand_pub = rospy.Publisher("reflex/pos_cmd", PoseCommand, queue_size=5)
        self.reward_pub = rospy.Publisher("agent/reward", Float32, queue_size=5)
        self.hand_sub = rospy.Subscriber("reflex/hand_state", Hand, self.hand_callback, queue_size=5)

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)

        # define system state
        self.hand_angles = np.zeros(4)
        self.hand_cmd = PoseCommand()

        # action for one finger (more, stay, less)
        self.action_space = gym.spaces.Box(low=np.array([-1]), high=np.array([1]), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=np.array([0]), high=np.array([2.3]), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

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

    def execute_for_seconds(self, secs, prefix):
        self.hand_pub.publish(self.hand_cmd)
        self.sim_unpause()
        rospy.loginfo(f"{prefix}: Executing {self.hand_cmd.f1} for {secs} secs.")
        rospy.sleep(secs)
        self.sim_pause()

    def step(self, action):
        # scale action from [-1,1] to [0,2]
        self.hand_cmd.f1 = action + 1
        self.execute_for_seconds(0.3, "Step")

        # obtain observations
        obs = np.zeros(self.observation_space.shape)
        obs[0] = self.hand_angles[0]
        # obs[1] = self.hand_angles[1]
        # obs[2] = self.hand_angles[2]
        # obs[3] = self.hand_angles[3]

        des_angle = 1
        reward = 0
        done = False
        logs = {}

        # reward should be 0 if on goal (scale linearly if not)
        reward = -1.0 * abs(self.hand_angles[0] - des_angle)
        reward_msg = Float32(reward)
        self.reward_pub.publish(reward_msg)

        limit_rad = 2
        limit_time = 8

        if obs[0] > limit_rad:
            done = True
            rospy.loginfo(f"Angle above {limit_rad} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > limit_time:
            done = True
            rospy.loginfo(f"Episode lasted {limit_time} secs. Setting done = True.")

        return obs, reward, done, logs

    def reset(self):
        # opening hand
        self.hand_cmd.f1 = 0
        self.execute_for_seconds(1, "Reset")

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")