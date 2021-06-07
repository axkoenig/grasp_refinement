from multiprocessing import Lock

import rospy

from .stage import Stage


class Rewards:
    def __init__(self, hparams, state):
        self.mutex = Lock()
        self.hparams = hparams
        self.state = state

    def get_reward_rlh(self, exec_secs=0.3):
        # computes reward during refining, lifting and holding
        return self.collect_reward(exec_secs)

    def get_reward_end(self):
        if self.hparams["framework"] == 1 or self.hparams["framework"] == 3:
            return self.hparams["w_binary_rew"] * int(self.state.sustained_holding)
        else:
            return 0

    def collect_reward(self, duration, rate=60):
        # records average reward over duration
        reward = 0
        counter = 0
        r = rospy.Rate(rate)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration.from_sec(duration):
            counter += 1
            reward += self.calc_reward()
            r.sleep()
        return reward / counter

    def calc_reward(self):
        with self.mutex:
            epsilons = self.state.epsilon_force + self.hparams["w_eps_torque"] * self.state.epsilon_torque
            delta = self.state.delta_task if self.state.stage == Stage.REFINE else self.state.delta_cur
        if self.hparams["framework"] == 1:
            return epsilons + self.hparams["w_delta"] * delta
        elif self.hparams["framework"] == 2:
            return delta
        elif self.hparams["framework"] == 3:
            return epsilons
        else:
            raise KeyError(f"Invalid framework number: {self.hparams['framework']}.")
