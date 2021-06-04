from stable_baselines3.common.callbacks import BaseCallback

from .helpers.logging import merge_dicts, log_dict


class TensorboardCallback(BaseCallback):
    def __init__(self, hparams, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)

        self.episode_infos = {}
        self.hparams = hparams

    def get_attr(self, name):
        # access local variable of training environment
        return self.locals[name][0]

    def get_done_or_dones(self): 
        # depending on which algorithm we're using we want to access "done" (e.g. td3) or "dones" (e.g. ppo)
        try_list = ["done", "dones"]
        for try_item in try_list:
            try: 
                return self.locals[try_item][0]
            except KeyError: 
                pass

    def _on_training_begin(self):
        log_dict(self.hparams, self.logger, "hparams/")

    def _on_step(self) -> bool:

        # log current metrics
        step_infos = self.get_attr("infos")
        log_dict(step_infos, self.logger, "step/", None, ["terminal_observation"])

        # save current metrics to compute sum later on
        self.episode_infos = merge_dicts(self.episode_infos, step_infos)

        # log cumulative values at episode end and reset variable
        if self.get_done_or_dones():
            exclude_cumulative_log = ["num_regrasps", "sustained_holding", "sustained_lifting", "terminal_observation"]
            log_dict(self.episode_infos, self.logger, "episode/cum_", "sum", exclude_cumulative_log)
            self.episode_infos = {}

        return True
