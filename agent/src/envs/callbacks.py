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
        self.cum_sum_contact_forces = 0

    def _on_rollout_end(self) -> None:
        self.logger.record("rollout/cum_num_contacts", self.cum_num_contacts)
        self.logger.record("rollout/cum_dist_tcp_obj", self.cum_dist_tcp_obj)
        self.logger.record("rollout/cum_epsilon_force", self.cum_epsilon_force)
        self.logger.record("rollout/cum_epsilon_torque", self.cum_epsilon_torque)
        self.logger.record("rollout/cum_obj_shift", self.cum_obj_shift)
        self.logger.record("rollout/cum_joint_diff", self.cum_joint_diff)
        self.logger.record("rollout/cum_delta_task", self.cum_delta_task)
        self.logger.record("rollout/cum_sum_contact_forces", self.cum_sum_contact_forces)

        # reset vars once recorded
        self.cum_num_contacts = 0
        self.cum_dist_tcp_obj = 0
        self.cum_epsilon_force = 0
        self.cum_epsilon_torque = 0
        self.cum_obj_shift = 0
        self.cum_joint_diff = 0
        self.cum_delta_task = 0
        self.cum_sum_contact_forces = 0

    def get_env_attr(self, name):
        return self.training_env.get_attr(name)[0]

    def _on_step(self) -> bool:
        self.cur_num_contacts = self.get_env_attr("num_contacts")
        self.cur_dist_tcp_obj = self.get_env_attr("dist_tcp_obj")
        self.cur_epsilon_force = self.get_env_attr("epsilon_force")
        self.cur_epsilon_torque = self.get_env_attr("epsilon_torque")
        self.cur_delta_task = self.get_env_attr("delta_task")
        self.cur_obj_shift = self.get_env_attr("obj_shift")
        self.cur_joint_diff = self.get_env_attr("prox_diff")
        self.cur_sum_contact_forces = self.get_env_attr("sum_contact_forces")

        self.logger.record("step/cur_num_contacts", self.cur_num_contacts)
        self.logger.record("step/cur_dist_tcp_obj", self.cur_dist_tcp_obj)
        self.logger.record("step/cur_epsilon_force", self.cur_epsilon_force)
        self.logger.record("step/cur_epsilon_torque", self.cur_epsilon_torque)
        self.logger.record("step/cur_delta_task", self.cur_delta_task)
        self.logger.record("step/cur_obj_shift", self.cur_obj_shift)
        self.logger.record("step/cur_joint_diff", self.cur_joint_diff)
        self.logger.record("step/cur_sum_contact_forces", self.cur_sum_contact_forces)

        self.cum_num_contacts += self.cur_num_contacts
        self.cum_dist_tcp_obj += self.cur_dist_tcp_obj
        self.cum_epsilon_force += self.cur_epsilon_torque
        self.cum_epsilon_torque += self.cur_epsilon_force
        self.cum_obj_shift += self.cur_obj_shift
        self.cum_joint_diff += self.cur_joint_diff
        self.cum_delta_task += self.cur_delta_task
        self.cum_sum_contact_forces += self.cur_sum_contact_forces

        # on episode end
        if self.get_env_attr("done"):
            self.logger.record("episode/num_regrasps", self.get_env_attr("num_regrasps"))
            self.logger.record("drop_test/delta_cur_lifting", self.get_env_attr("delta_cur_lifting"))
            self.logger.record("drop_test/delta_cur_holding", self.get_env_attr("delta_cur_holding"))
            self.logger.record("drop_test/eps_force_lifting", self.get_env_attr("eps_force_lifting"))
            self.logger.record("drop_test/eps_force_holding", self.get_env_attr("eps_force_holding"))
            self.logger.record("drop_test/eps_torque_lifting", self.get_env_attr("eps_torque_lifting"))
            self.logger.record("drop_test/eps_torque_holding", self.get_env_attr("eps_torque_holding"))
            self.logger.record("drop_test/sustained_lifting", self.get_env_attr("sustained_lifting"))
            self.logger.record("drop_test/sustained_holding", self.get_env_attr("sustained_holding"))

        return True
