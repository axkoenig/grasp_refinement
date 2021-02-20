import gym
import numpy as np

import rospy
from std_msgs.msg import Float32
from reflex_msgs.msg import PoseCommand, Hand

from .helpers import rad2deg, deg2rad, get_homo_matrix_from_tq
from .space import Space
from .gazebo_interface import GazeboInterface

# TODO clip observation_space if needed and alert (check if Gym does this by default)
# TODO experiment stopping CONDITIONS end if object too further than max wrist obj distance away
# TODO check what seed does


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
        self.add_variable(self.num_contact_pressures, "contact_pressure", 0, 0, 5)
        self.add_variable(self.num_wrist_obj_dists, "wrist_obj_dist", 0, 0, 0.2)


class ActionSpace(Space):
    """Defines action space"""

    def __init__(self):
        super().__init__()

        self.add_variable(3, "finger_incr", 0, deg2rad(-10), deg2rad(10))
        self.add_variable(1, "wrist_z_abs", 0, 0, 0.06)


class GazeboEnv(gym.Env):
    def __init__(self, exec_secs, max_ep_len, joint_lim):

        self.exec_secs = exec_secs
        self.max_ep_len = max_ep_len
        self.joint_lim = joint_lim
        self.epsilon_scaling = 10

        rospy.init_node("agent", anonymous=True)

        self.wrist_init_pose = get_homo_matrix_from_tq(
            [-0.04029641827077922, 0.1877200198173525, 0.08613854642685925],
            [-0.7072138021849631, 2.9635822779245583e-05, 2.9644795664464442e-05, 0.7069997427453507],
        )
        self.obj_init_pose = get_homo_matrix_from_tq(
            [-0.03029641827077922, 0.3857962704198499, 0.07269308204115527],
            [0.08724778936721149, -0.1950795486820534, -0.026497115718354176, 0.9765396539798828],
        )

        self.gazebo_interface = GazeboInterface()
        self.hand_cmd = PoseCommand()
        self.acts = ActionSpace()
        self.obs = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals(), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals(), dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)

        self.hand_pub = rospy.Publisher("reflex/pos_cmd", PoseCommand, queue_size=5)
        self.reward_pub = rospy.Publisher("agent/reward", Float32, queue_size=5)
        self.hand_sub = rospy.Subscriber("reflex/hand_state", Hand, self.hand_callback, queue_size=5)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def hand_callback(self, msg):

        for i in range(self.obs.num_fingers):
            # joint pos 1 to 3 from magnetic encoders (more accurate than motor encoders)
            self.obs.vars[i].cur_val = msg.finger[i].proximal
            self.obs.vars[i + self.obs.num_motors].cur_val = msg.finger[i].distal_approx

            for j in range(self.obs.num_sensors):
                self.obs.vars[i + self.obs.num_motors + self.obs.num_fingers + j].cur_val = msg.finger[i].pressure[j]

        # joint pos 4 from motor encoder (there is no magnetic encoder)
        self.obs.vars[3].cur_val = msg.motor[3].joint_angle

        # last three variables are obj position in tcp frame
        trans = self.gazebo_interface.get_trans_tcp_obj()
        self.obs.vars[-3].cur_val = trans[0]
        self.obs.vars[-2].cur_val = trans[1]
        self.obs.vars[-1].cur_val = trans[2]

    def step(self, action):

        # publish incoming actions
        self.hand_cmd.f1 = self.obs.vars[0].cur_val + action[0]
        self.hand_cmd.f2 = self.obs.vars[1].cur_val + action[1]
        self.hand_cmd.f3 = self.obs.vars[2].cur_val + action[2]
        self.hand_cmd.preshape = 1.570796
        self.hand_pub.publish(self.hand_cmd)
        self.gazebo_interface.move_wrist_along_z(action[3])

        cmd_str = f"{self.hand_cmd.f1}, {self.hand_cmd.f2}, {self.hand_cmd.f3}, {action[3]}"
        self.gazebo_interface.run_for_seconds("Step", self.exec_secs, cmd_str)

        reward = 0
        done = False
        logs = {}

        reward_msg = Float32(reward)
        self.reward_pub.publish(reward_msg)

        prox_angles = self.obs.vars[:3]
        if not all(prox_angle.cur_val < self.joint_lim for prox_angle in prox_angles):
            done = True
            rospy.loginfo(f"One angle is above {self.joint_lim} rad. Setting done = True.")
        elif rospy.get_rostime().secs - self.last_reset_time.secs > self.max_ep_len:
            done = True
            rospy.loginfo(f"Episode lasted {self.max_ep_len} secs. Setting done = True.")

        return self.obs.get_cur_vals(), reward, done, logs

    def reset(self):
        self.gazebo_interface.reset_world(self.wrist_init_pose, self.obj_init_pose)

        # reset vars
        obs = np.zeros(self.observation_space.shape)
        self.last_reset_time = rospy.get_rostime()
        return obs

    def close(self):
        rospy.signal_shutdown("Gym is now closing.")
        rospy.loginfo("Done! Have a nice day.")
