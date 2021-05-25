import numpy as np

from .space import Space


class ObservationSpace(Space):
    """Defines observation space."""

    def __init__(self):
        super().__init__()

        self.num_fingers = 3
        self.num_sensors = 9
        self.prox_angle_max = 3
        self.preshape_angle_max = np.pi / 2
        self.tactile_pos_default = [0, 0, 0.06]

        for i in range(self.num_fingers):
            id_str = "_f" + str(i + 1)
            self.add_variable(1, "prox_angle" + id_str, 0, 0, self.prox_angle_max)
            self.add_variable(1, "dist_angle" + id_str, 0, 0, 0.2 * self.prox_angle_max)
            self.add_variable(1, "prox_normal" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])
            self.add_variable(1, "dist_normal" + id_str, [0, 0, 0], [-1, -1, -1], [1, 1, 1])
            self.add_variable(1, "tactile_position" + id_str + "_prox", self.tactile_pos_default, [-0.2, -0.16, 0.06], [0.2, 0.16, 0.2])
            self.add_variable(1, "tactile_position" + id_str + "_dist", self.tactile_pos_default, [-0.2, -0.16, 0.06], [0.2, 0.16, 0.2])

            for j in range(self.num_sensors):
                id_str = "_f" + str(i + 1) + "_s" + str(j + 1)
                self.add_variable(1, "sensor_pressure" + id_str, 0, 0, 127)
                self.add_variable(1, "tactile_contact" + id_str, 0, 0, 1)

        self.add_variable(1, "preshape_angle", 0, 0, self.preshape_angle_max)

        self.print_num_dimensions()