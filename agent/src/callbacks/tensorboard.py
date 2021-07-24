from stable_baselines3.common.callbacks import BaseCallback

from controller.helpers.logging import log_dict


class TensorboardCallback(BaseCallback):
    def __init__(self, hparams, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)

        self.episode_infos = {}
        self.hparams = hparams

    def get_attr(self, name):
        # access local variable of training environment
        return self.locals[name][0]

    def _on_training_start(self):
        log_dict(self.hparams, self.logger, "hparams/")

    def _on_step(self) -> bool:
        log_dict(self.get_attr("infos"), self.logger, "step/", None, ["terminal_observation"])

        return True
