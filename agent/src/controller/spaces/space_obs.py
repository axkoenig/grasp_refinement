import numpy as np

from .space import Space


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.num_fingers = 3
        self.num_parts = 4  # 1 palm and 3 fingers
        self.num_sensors = 9

        self.prox_angle_max = 3
        self.motor_torque_max = 5
        self.motor_torque_min = -0.2
        self.preshape_angle_max = np.pi / 2

        self.tactile_pos_min = [-0.2, -0.2, 0.05]
        self.tactile_pos_max = [0.2, 0.2, 0.26]
        self.tactile_pos_default = [0, 0, 0.06]

        self.contact_force_min = [-15, -15, -15]
        self.contact_force_max = [15, 15, 15]
        self.contact_force_default = [0, 0, 0]

        self.contact_normal_min = [-1, -1, -1]
        self.contact_normal_max = [1, 1, 1]
        self.contact_normal_default = [0, 0, 0]

        # information obtainable from real hand
        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable(1, "prox_angle" + id_str, 0, 0, self.prox_angle_max)
            self.add_variable(1, "dist_angle" + id_str, 0, 0, 0.2 * self.prox_angle_max)
            self.add_variable(1, "motor_torque" + id_str, 0, self.motor_torque_min, self.motor_torque_max)
        self.add_variable(1, "preshape_angle", 0, 0, self.preshape_angle_max)
        self.add_variable(1, "preshape_motor_torque", 0, self.motor_torque_min, self.motor_torque_max)

        # information available only in simulation
        for i in range(self.num_parts):
            id_str = "_p" + str(i)
            # palm
            if i == 0:
                self.add_variable(1, "contact_normal" + id_str, self.contact_normal_default, self.contact_normal_min, self.contact_normal_max)
                self.add_variable(1, "contact_pos" + id_str, self.tactile_pos_default, self.tactile_pos_min, self.tactile_pos_max)
                self.add_variable(1, "contact_force" + id_str, self.contact_force_default, self.contact_force_min, self.contact_force_max)
                continue
            # fingers
            for link in ["_prox", "_dist"]:
                self.add_variable(1, "contact_normal" + id_str + link, self.contact_normal_default, self.contact_normal_min, self.contact_normal_max)
                self.add_variable(1, "contact_pos" + id_str + link, self.tactile_pos_default, self.tactile_pos_min, self.tactile_pos_max)
                self.add_variable(1, "contact_force" + id_str + link, self.contact_force_default, self.contact_force_min, self.contact_force_max)

        self.print_num_dimensions()

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
