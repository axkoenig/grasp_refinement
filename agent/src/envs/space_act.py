import numpy as np

from .space import Space
from .helpers import deg2rad


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.max_wrist_trans = 0.01
        self.min_wrist_trans = -self.max_wrist_trans
        self.max_wrist_rot = deg2rad(5)
        self.min_wrist_rot = -self.max_wrist_rot
        self.max_finger_incr = deg2rad(2)
        self.min_finger_incr = -self.max_finger_incr / 4

        self.max_action = 1
        self.min_action = -self.max_action

        self.add_variable(11, "actions", 0, self.min_action, self.max_action)
        self.print_num_dimensions()

    def map_vals_to_range(self, min, max, vals):
        "Maps each value in a list of values in range [self.min_action, self.max_action] to [min, max]"
        if not min < max:
            raise ValueError("Your 'min' must be less than your 'max'.")
        for i in range(len(vals)):
            vals[i] = np.interp(vals[i], (self.min_action, self.max_action), (min, max))
        return vals

    def get_action_dict(self, action, verbose=True):
        "Converts action array from gym environment to a more expressive dict with correct ranges"
        action_dict = {
            "trigger_regrasp": True if action[0] > 0 else False,
            "wrist_trans": self.map_vals_to_range(self.min_wrist_trans, self.max_wrist_trans, action[1:4]),
            "wrist_rot": self.map_vals_to_range(self.min_wrist_rot, self.max_wrist_rot, action[4:7]),
            "trigger_fingers": True if action[7] > 0 else False,
            "fingers_incr": self.map_vals_to_range(self.min_finger_incr, self.max_finger_incr, action[8:11]),
        }
        if verbose:
            for key, value in action_dict.items():
                print(f'- {key:20}{value}')
        return action_dict
