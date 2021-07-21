import gym
import rospy
import numpy as np

from .helpers.logging import get_infos
from .spaces.space_act import ActionSpace
from .spaces.space_obs import ObservationSpace
from .gazebo_interface import GazeboInterface
from .subscribers import Subscribers
from .stage import Stage, StageController
from .state import State
from .rewards import Rewards
from .actions import Actions
from .writer import Writer


class Controller(gym.Env):
    def __init__(self, hparams, name="TRAIN"):

        self.hparams = hparams
        self.name = name

        self.state = State()
        self.gi = GazeboInterface(self.hparams, verbose=False)
        self.acts = ActionSpace()
        self.obs = ObservationSpace(self.hparams)
        self.sub = Subscribers(self.hparams, self.state, self.obs, self.gi)
        self.rewards = Rewards(self.hparams, self.state)
        self.stage_controller = StageController(self.hparams, self.state, self.gi)
        self.actions = Actions(self.hparams, self.state, self.gi)
        self.writer = Writer(self.hparams)

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals())
        self.observation_space = gym.spaces.Box(low=np.zeros(self.obs.get_min_vals().shape), high=np.ones(self.obs.get_max_vals().shape))
        self.reward_range = (0, 1)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-{self.state.stage.name}-STEP:[{self.state.cur_time_step}]===")

        action_dict = self.acts.get_action_dict(action, verbose=True)
        self.actions.act(action_dict)

        reward = self.rewards.get_reward_rlh(self.actions.get_rate_of_cur_stage())

        self.stage_controller.update_stage()
        self.done = True if self.state.stage == Stage.END else False

        if self.done:
            reward += self.rewards.get_reward_end()

        rospy.loginfo(f"--> REWARD: \t {reward}")

        obs_dict = self.obs.get_cur_vals()
        infos_dict = get_infos(self.state)
        self.state.store_io_in_buffer(obs_dict, action_dict, reward, infos_dict)

        return np.array(list(obs_dict.values())), reward, self.done, infos_dict

    def reset(self, test_case=None):
        self.gi.sim_unpause()
        self.writer.write(self.state.io_buffer, self.name)
        rospy.loginfo(f"==={self.name}-RESETTING===")
        self.gi.reset_world(self.state, test_case)
        self.state.reset()
        self.gi.sim_pause()  # when NN is updating after resetting, we pause simulation
        return np.array(list(self.obs.get_cur_vals().values()))

    def close(self):
        self.writer.write(self.state.io_buffer, self.name)
        self.state.reset()