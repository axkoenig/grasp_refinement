import numpy as np

from .space import Space


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.num_fingers = 3
        self.num_sensors = 9
        self.prox_angle_max = 3
        self.motor_torque_max = 5
        self.preshape_angle_max = np.pi / 2
        self.tactile_pos_default = [0, 0, 0.06]
        self.contact_force_min = [-10, -10, -10]
        self.contact_force_max = [10, 10, 10]

        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable(1, "motor_torque" + id_str, 0, 0, self.motor_torque_max)
            self.add_variable(1, "prox_angle" + id_str, 0, 0, self.prox_angle_max)
            self.add_variable(1, "dist_angle" + id_str, 0, 0, 0.2 * self.prox_angle_max)
            self.add_variable(1, "contact_normal_prox" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])
            self.add_variable(1, "contact_normal_dist" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])
            self.add_variable(1, "contact_pos_prox" + id_str, self.tactile_pos_default, [-0.2, -0.16, 0.06], [0.2, 0.16, 0.2])
            self.add_variable(1, "contact_pos_dist" + id_str, self.tactile_pos_default, [-0.2, -0.16, 0.06], [0.2, 0.16, 0.2])
            self.add_variable(1, "contact_force_prox" + id_str, [0, 0, 0], self.contact_force_min, self.contact_force_max)
            self.add_variable(1, "contact_force_dist" + id_str, [0, 0, 0], self.contact_force_min, self.contact_force_max)

        self.add_variable(1, "preshape_angle", 0, 0, self.preshape_angle_max)
        self.add_variable(1, "preshape_motor_torque", 0, 0, self.motor_torque_max)

        self.print_num_dimensions()