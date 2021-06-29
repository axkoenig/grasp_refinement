from multiprocessing import Lock

import numpy as np

from .space import Space


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self, hparams):
        super().__init__()
        self.hparams = hparams

        # this info comes from reflex urdf
        shell_box_x = 0.13
        shell_box_y = 0.084
        shell_box_z = 0.07

        self.num_fingers = 3
        self.num_parts = 4  # 1 palm and 3 fingers
        self.num_sensors = 9

        self.prox_angle_max = 3
        self.joint_angle_min = -0.01
        self.motor_torque_max = 10
        self.motor_torque_min = -3
        self.preshape_angle_max = np.pi / 2

        # finger contact limits
        self.f_contact_pos_min = [-0.2, -0.3, 0.05]
        self.f_contact_pos_max = [0.2, 0.3, 0.3]
        self.f_contact_pos_default = [0, 0, shell_box_z - 0.02]
        self.f_contact_normal_min = [-1, -1, -1]
        self.f_contact_normal_max = [1, 1, 1]
        self.f_contact_normal_default = [0, 0, 0]

        # palm contact limits
        self.p_contact_pos_min = [-shell_box_x / 2, -shell_box_y / 2, shell_box_z - 0.01]
        self.p_contact_pos_max = [shell_box_x / 2, shell_box_y / 2, shell_box_z + 0.01]
        self.p_contact_pos_default = [0, 0, shell_box_z]
        self.p_contact_normal_min = [-1, -1, 0]  # normal always points away from palm
        self.p_contact_normal_max = [1, 1, 1]
        self.p_contact_normal_default = [0, 0, 1]

        # information obtainable from real hand
        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable(1, "prox_angle" + id_str, 0, self.joint_angle_min, self.prox_angle_max)
            self.add_variable(1, "dist_angle" + id_str, 0, self.joint_angle_min, 0.2 * self.prox_angle_max)
            self.add_variable(1, "motor_torque" + id_str, 0, self.motor_torque_min, self.motor_torque_max)
        self.add_variable(1, "preshape_angle", 0, 0, self.preshape_angle_max)
        self.add_variable(1, "preshape_motor_torque", 0, self.motor_torque_min, self.motor_torque_max)

        # if no contact sensing, we're done
        if self.hparams["force_sensing"] == 4:
            self.print_num_dimensions()
            return

        # contact information available only in simulation
        for i in range(self.num_parts):
            id_str = "_p" + str(i)
            # palm
            if i == 0:
                self.add_variable(1, "contact_normal" + id_str, self.p_contact_normal_default, self.p_contact_normal_min, self.p_contact_normal_max)
                self.add_variable(1, "contact_pos" + id_str, self.p_contact_pos_default, self.p_contact_pos_min, self.p_contact_pos_max)
                self.add_force_sensing("contact_force" + id_str)
                continue
            # fingers
            for link in ["_prox", "_dist"]:
                self.add_variable(1, "contact_normal" + id_str + link, self.f_contact_normal_default, self.f_contact_normal_min, self.f_contact_normal_max)
                self.add_variable(1, "contact_pos" + id_str + link, self.f_contact_pos_default, self.f_contact_pos_min, self.f_contact_pos_max)
                self.add_force_sensing("contact_force" + id_str + link)

        self.print_num_dimensions()

    def add_force_sensing(self, var_name):
        if self.hparams["force_sensing"] == 1:  # full force vector
            self.contact_force_min = [-15, -15, -15]
            self.contact_force_max = [15, 15, 15]
            self.contact_force_default = [0, 0, 0]
            self.add_variable(1, var_name, self.contact_force_default, self.contact_force_min, self.contact_force_max)
        elif self.hparams["force_sensing"] == 2:  # only normal force
            self.contact_force_min = 0
            self.contact_force_max = 15
            self.contact_force_default = 0
            self.add_variable(1, var_name, self.contact_force_default, self.contact_force_min, self.contact_force_max)
        elif self.hparams["force_sensing"] == 3:  # only binary contact signals
            self.contact_force_min = 0
            self.contact_force_max = 1
            self.contact_force_default = 0
            self.add_variable(1, var_name, self.contact_force_default, self.contact_force_min, self.contact_force_max)
        else:
            raise ValueError("Unsupported force sensing framework.")

    def reset_contact_obs(self):
        for i in range(self.num_parts):
            id_str = "_p" + str(i)
            # palm
            if i == 0:
                self.set_cur_val_by_name("contact_normal" + id_str, self.contact_normal_default)
                self.set_cur_val_by_name("contact_pos" + id_str, self.tactile_pos_default)
                self.set_cur_val_by_name("contact_force" + id_str, self.contact_force_default)
                continue
            # fingers
            for link in ["_prox", "_dist"]:
                self.set_cur_val_by_name("contact_normal" + id_str + link, self.contact_normal_default)
                self.set_cur_val_by_name("contact_pos" + id_str + link, self.tactile_pos_default)
                self.set_cur_val_by_name("contact_force" + id_str + link, self.contact_force_default)
