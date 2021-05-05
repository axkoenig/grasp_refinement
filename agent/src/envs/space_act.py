from .space import Space


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.add_variable(1, "trigger_regrasp", 0, 0, 1)
        self.add_variable(3, "wrist_trans", 0, -0.01, 0.01)
        self.add_variable(3, "wrist_rot", 0, -0.08, 0.08)
        self.add_variable(3, "trigger_finger_tightening", 0, 0, 1)

        self.print_num_dimensions()
