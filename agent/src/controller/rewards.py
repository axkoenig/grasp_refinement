from multiprocessing import Lock

import rospy

from .stage import Stage


class Rewards:
    def __init__(self, hparams, state):
        self.mutex = Lock()
        self.hparams = hparams
        self.state = state

    def get_reward_rlh(self, rate_of_cur_stage):
        # computes reward during refining, lifting and holding
        return self.collect_reward(self.get_exec_secs(rate_of_cur_stage))

    def get_reward_end(self):
        if self.hparams["framework"] == 4:
            return int(self.state.sustained_holding)
        else:
            return 0

    def get_exec_secs(self, rate_of_cur_stage):
        # this is to wait 80% of one time step to collect reward
        # we leave 20% for other code to run (later on we make sure we get the exact update rate via the wait_if_necessary method)
        return 0.8 * (1 / rate_of_cur_stage)

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
            epsilons = self.normalize(epsilons, 0, 1 + self.hparams["w_eps_torque"])
            delta = self.state.delta_task if self.state.stage == Stage.REFINE else self.state.delta_cur
        if self.hparams["framework"] == 1:
            return epsilons + self.hparams["w_delta"] * delta
        elif self.hparams["framework"] == 2:
            return delta
        elif self.hparams["framework"] == 3:
            return epsilons
        elif self.hparams["framework"] == 4:
            return 0  # will only get binary reward at end
        else:
            raise KeyError(f"Invalid framework number: {self.hparams['framework']}.")

    def normalize(self, val, low_bound, high_bound):
        return (val - low_bound) / (high_bound - low_bound)
