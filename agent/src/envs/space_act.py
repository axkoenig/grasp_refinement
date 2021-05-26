import numpy as np
import rospy

from .space import Space
from .helpers import deg2rad


class ActionSpace(Space):
    """Defines action space."""

    def __init__(self):
        super().__init__()

        self.max_wrist_trans = 0.02
        self.min_wrist_trans = -self.max_wrist_trans
        self.max_wrist_rot = deg2rad(3)
        self.min_wrist_rot = -self.max_wrist_rot
        self.max_finger_incr = deg2rad(3)
        self.min_finger_incr = deg2rad(-1)

        self.max_action = 1
        self.min_action = -self.max_action

        self.add_variable(10, "actions", 0, self.min_action, self.max_action)
        self.print_num_dimensions()

    def map_vals_to_range(self, min, max, vals):
        "Maps each value in a list of values in range [self.min_action, self.max_action] to [min, max]"
        if not min < max:
            raise ValueError("Your 'min' must be less than your 'max'.")
        try: 
            for i in range(len(vals)):
                vals[i] = np.interp(vals[i], (self.min_action, self.max_action), (min, max))
            return vals
        except TypeError: # we only have one value
            return np.interp(vals, (self.min_action, self.max_action), (min, max))

    def get_action_dict(self, action, verbose=True):
        "Converts action array from gym environment to a more expressive dict with correct ranges"
        
        # TODO remove try catch (this should not be necessary)
        # try: 
        #     trigger_regrasp = np.random.binomial(1, self.map_vals_to_range(0, 1, action[0]))
        # except ValueError as e:
        #     rospy.logwarn(f"You action is {action[0]} and raised a ValueError: '{e}'. Retrying with more strict bounds.")
        #     trigger_regrasp = np.random.binomial(1, self.map_vals_to_range(0.00001, 0.999999, action[0]))

        action_dict = {
            "trigger_regrasp": True if action[0] > 0 else False,
            "wrist_trans": self.map_vals_to_range(self.min_wrist_trans, self.max_wrist_trans, action[1:4]),
            "wrist_rot": self.map_vals_to_range(self.min_wrist_rot, self.max_wrist_rot, action[4:7]),
            "fingers_incr": self.map_vals_to_range(self.min_finger_incr, self.max_finger_incr, action[7:10]),
        }
        if verbose:
            rospy.loginfo("--- Actions ---")
            for key, value in action_dict.items():
                if key == "trigger_regrasp":
                    rospy.loginfo(f'- {key:20}{value} sampled from: {self.map_vals_to_range(0, 1, action[0])}')
                else:
                    rospy.loginfo(f'- {key:20}{value}')
        return action_dict
