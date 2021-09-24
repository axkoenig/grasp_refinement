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
        self.nan_array = [np.nan, np.nan, np.nan]

        # finger joint limits
        self.prox_angle_max = 3
        self.joint_angle_min = -0.01
        self.preshape_angle_max = np.pi / 2
        self.preshape_angle_min = 0
        self.motor_torque_max = 10
        self.motor_torque_min = -3

        # finger contact limits
        self.f_contact_pos_min = [-0.2, -0.3, 0.05]
        self.f_contact_pos_max = [0.2, 0.3, 0.3]
        self.f_contact_normal_min = [-1, -1, -1]
        self.f_contact_normal_max = [1, 1, 1]

        # palm contact limits
        self.p_contact_pos_min = [-shell_box_x / 2 - 0.03, -shell_box_y / 2 - 0.03, shell_box_z - 0.03]
        self.p_contact_pos_max = [shell_box_x / 2 + 0.03, shell_box_y / 2 + 0.03, shell_box_z + 0.03]
        self.p_contact_normal_min = [-1, -1, 0]  # normal always points away from palm
        self.p_contact_normal_max = [1, 1, 1]

        # information obtainable from real hand
        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable("prox_angle" + id_str, self.joint_angle_min, self.prox_angle_max)
            self.add_variable("dist_angle" + id_str, self.joint_angle_min, 0.2 * self.prox_angle_max)
            if self.hparams["torque_framework"] != 3:
                self.add_variable("motor_torque" + id_str, self.motor_torque_min, self.motor_torque_max)
        self.add_variable("preshape_angle", self.preshape_angle_min, self.preshape_angle_max)
        if self.hparams["torque_framework"] != 3:
            self.add_variable("preshape_motor_torque", self.motor_torque_min, self.motor_torque_max)

        # if no contact sensing, we're done
        if self.hparams["force_framework"] == 4:
            self.print_num_dimensions()
            return

        # contact information available only in simulation
        for i in range(self.num_parts):
            id_str = "_p" + str(i)
            # palm
            if i == 0:
                self.add_variable_from_vector("contact_normal" + id_str, self.p_contact_normal_min, self.p_contact_normal_max)
                self.add_variable_from_vector("contact_pos" + id_str, self.p_contact_pos_min, self.p_contact_pos_max)
                self.add_force_sensing("contact_force" + id_str)
                continue
            # fingers
            for link in ["_prox", "_dist"]:
                self.add_variable_from_vector("contact_normal" + id_str + link, self.f_contact_normal_min, self.f_contact_normal_max)
                self.add_variable_from_vector("contact_pos" + id_str + link, self.f_contact_pos_min, self.f_contact_pos_max)
                self.add_force_sensing("contact_force" + id_str + link)

        self.print_num_dimensions()

    def add_force_sensing(self, var_name, force_max=20):
        if self.hparams["force_framework"] == 1:  # full force vector
            self.contact_force_min = [-force_max, -force_max, -force_max]
            self.contact_force_max = [force_max, force_max, force_max]
            self.add_variable_from_vector(var_name, self.contact_force_min, self.contact_force_max)
        elif self.hparams["force_framework"] == 2:  # only normal force
            self.contact_force_min = -force_max  # in reality normal force can only be positive, but DART sometimes also outputs negative normal forces
            self.contact_force_max = force_max
            self.add_variable(var_name, self.contact_force_min, self.contact_force_max)
        elif self.hparams["force_framework"] == 3:  # only binary contact signals
            self.contact_force_min = 0
            self.contact_force_max = 1
            self.add_variable(var_name, self.contact_force_min, self.contact_force_max)

    def reset_contact_obs(self):
        for i in range(self.num_parts):
            id_str = "_p" + str(i)
            # palm
            if i == 0:
                self.set_variable_from_vector("contact_normal" + id_str, self.nan_array)
                self.set_variable_from_vector("contact_pos" + id_str, self.nan_array)
                if self.hparams["force_framework"] == 1:
                    self.set_variable_from_vector("contact_force" + id_str, self.nan_array)
                else:
                    self.set_variable("contact_force" + id_str, np.nan)
                continue
            # fingers
            for link in ["_prox", "_dist"]:
                self.set_variable_from_vector("contact_normal" + id_str + link, self.nan_array)
                self.set_variable_from_vector("contact_pos" + id_str + link, self.nan_array)
                if self.hparams["force_framework"] == 1:
                    self.set_variable_from_vector("contact_force" + id_str + link, self.nan_array)
                else:
                    self.set_variable("contact_force" + id_str + link, np.nan)
