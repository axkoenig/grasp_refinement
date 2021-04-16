from stable_baselines3.common.callbacks import BaseCallback


class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0
        self.cum_joint_diff = 0
        self.cum_delta_task = 0

    def _on_rollout_end(self) -> None:
        self.logger.record("rollout/cum_num_contacts", self.cum_num_contacts)
        self.logger.record("rollout/cum_dist_tcp_obj", self.cum_dist_tcp_obj)
        self.logger.record("rollout/cum_epsilon_force", self.cum_epsilon_force)
        self.logger.record("rollout/cum_epsilon_torque", self.cum_epsilon_torque)
        self.logger.record("rollout/cum_obj_shift", self.cum_obj_shift)
        self.logger.record("rollout/cum_joint_diff", self.cum_joint_diff)
        self.logger.record("rollout/cum_delta_task", self.cum_delta_task)

        # reset vars once recorded
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0
        self.cum_joint_diff = 0
        self.cum_delta_task = 0

    def _on_step(self) -> bool:
        self.cur_num_regrasps = self.training_env.get_attr("num_regrasps")[0]
        self.cur_num_contacts = self.training_env.get_attr("num_contacts")[0]
        self.cur_dist_tcp_obj = self.training_env.get_attr("dist_tcp_obj")[0]
        self.cur_epsilon_force = self.training_env.get_attr("epsilon_force")[0]
        self.cur_epsilon_torque = self.training_env.get_attr("epsilon_torque")[0]
        self.cur_delta_task = self.training_env.get_attr("delta_task")[0]
        self.cur_obj_shift = self.training_env.get_attr("obj_shift")[0]
        self.cur_joint_diff = self.training_env.get_attr("prox_diff")[0]

        self.logger.record("step/cur_num_regrasps", self.cur_num_regrasps)
        self.logger.record("step/cur_num_contacts", self.cur_num_contacts)
        self.logger.record("step/cur_dist_tcp_obj", self.cur_dist_tcp_obj)
        self.logger.record("step/cur_epsilon_force", self.cur_epsilon_force)
        self.logger.record("step/cur_epsilon_torque", self.cur_epsilon_torque)
        self.logger.record("step/cur_delta_task", self.cur_delta_task)
        self.logger.record("step/cur_obj_shift", self.cur_obj_shift)
        self.logger.record("step/cur_joint_diff", self.cur_joint_diff)

        self.cum_num_contacts += self.cur_num_contacts
        self.cum_dist_tcp_obj += self.cur_dist_tcp_obj
        self.cum_epsilon_force += self.cur_epsilon_torque
        self.cum_epsilon_torque += self.cur_epsilon_force
        self.cum_obj_shift += self.cur_obj_shift
        self.cum_joint_diff += self.cur_joint_diff
        self.cum_delta_task += self.cur_delta_task
        return True
