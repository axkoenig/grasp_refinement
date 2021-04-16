from .space import Space


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.add_variable(1, "trigger_regrasp", 0, 0, 1)
        self.add_variable(1, "wrist_incr_x", 0, -0.01, 0.01)
        self.add_variable(1, "wrist_incr_y", 0, -0.001, 0.001)
        self.add_variable(1, "wrist_incr_z", 0, -0.005, 0.02)
        self.add_variable(1, "wrist_pitch", 0, -0.1, 0.1)
        self.add_variable(3, "trigger_finger_tightening", 0, 0, 1)
