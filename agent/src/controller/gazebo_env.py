from multiprocessing import Lock

import gym
import numpy as np
import rospy
import tf

from .helpers.logging import get_infos
from .spaces.space_act import ActionSpace
from .spaces.space_obs import ObservationSpace
from .gazebo_interface import GazeboInterface
from .subscribers import Subscribers
from .stage import Stage, StageController
from .state import State
from .rewards import Rewards


class GazeboEnv(gym.Env):
    def __init__(self, hparams, name="TRAIN"):

        self.hparams = hparams
        self.name = name
        self.mutex = Lock()

        self.state = State()
        self.gi = GazeboInterface(verbose=False)
        self.acts = ActionSpace()
        self.obs = ObservationSpace()
        self.sub = Subscribers(self.state, self.obs, self.gi)
        self.rewards = Rewards(self.hparams, self.state)
        self.stage_controller = StageController(self.hparams, self.state, self.gi)

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals())
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals())
        self.reward_range = (-np.inf, np.inf)

        # update rates
        self.rate_lift = self.hparams["lift_steps"] / self.hparams["secs_to_lift"]
        self.rate_hold = self.hparams["hold_steps"] / self.hparams["secs_to_hold"]
        rospy.loginfo("Rate lift is: \t%f", self.rate_lift)
        rospy.loginfo("Rate hold is: \t%f", self.rate_hold)

    ### GYM METHODS

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-{self.state.stage.name}-STEP:[{self.state.cur_time_step}]===")

        action_dict = self.acts.get_action_dict(action, verbose=True)
        self.act(action_dict)

        reward = self.rewards.get_reward_rlh(self.get_exec_secs())

        self.stage_controller.update_stage()
        self.done = True if self.state.stage == Stage.END else False

        if self.done:
            reward += self.get_reward_end()

        rospy.loginfo(f"--> REWARD: \t {reward}")

        return self.obs.get_cur_vals(), reward, self.done, get_infos(self.state)

    def reset(self):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-RESETTING===")
        self.gi.reset_world(self.hparams)
        self.last_time_stamp = rospy.Time.now()
        self.state.reset()
        self.gi.sim_pause()  # when NN is updating after resetting, we pause simulation
        return self.obs.get_cur_vals()

    ### OTHER METHODS

    def get_exec_secs(self):
        # this is to wait 80% of one time step to collect reward
        # we leave 20% for other code to run (later on we make sure we get the exact update rate via the wait_if_necessary method)
        return 0.8 * (1 / self.get_rate_of_cur_stage())

    def act(self, action_dict):
        if action_dict["trigger_regrasp"]:
            # we only allow wrist control during refinement
            if self.state.stage == Stage.REFINE:
                rospy.loginfo(">>REGRASPING<<")
                rot = action_dict["wrist_rot"]
                wrist_q = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
                self.gi.regrasp(action_dict["wrist_trans"], wrist_q, self.state.prox_angles)
                self.state.num_regrasps += 1
            else:
                rospy.loginfo(">>STAYING<<")
        else:
            rospy.loginfo(">>ADJUSTING FINGERS<<")
            self.wait_if_necessary()
            f = action_dict["fingers_incr"]
            self.gi.pos_incr(f[0], f[1], f[2], 0, False, False, 0, 0)

    def get_rate_of_cur_stage(self):
        if self.state.stage == Stage.REFINE:
            return self.hparams["max_refine_rate"]
        elif self.state.stage == Stage.LIFT:
            return self.rate_lift
        elif self.state.stage == Stage.HOLD:
            return self.rate_hold
        else:
            # there is no rate at stage END because it's only one time step
            return 1

    def wait_if_necessary(self):
        # makes sure that we are keeping desired update rate
        step_size = 1 / self.get_rate_of_cur_stage()
        d = rospy.Time.now() - self.last_time_stamp
        while rospy.Time.now() - self.last_time_stamp < rospy.Duration(step_size):
            rospy.loginfo_throttle(
                step_size, "Your last %s step only took %f seconds. Waiting to keep min step size of %f", self.state.stage.name, d.to_sec(), step_size
            )
            rospy.sleep(0.01)
        self.last_time_stamp = rospy.Time.now()


