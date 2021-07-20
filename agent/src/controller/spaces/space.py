from multiprocessing import Lock
from copy import deepcopy

import numpy as np
import rospy


class Variable:
    """Defines one action or observation variable."""

    def __init__(self, name, min_val, max_val):
        self.name = name
        self.cur_val = np.nan
        self.min_val = min_val
        self.max_val = max_val


class Space:
    """Parent class for observation and action class."""

    def __init__(self):
        self.vars = {}
        self.mutex = Lock()
        self.dim = 0
        self.vector_length = 3
        self.vector_suffixes = ["_x", "_y", "_z"]

    def add_variable_from_vector(self, name, min_vals, max_vals):
        if len(min_vals) != self.vector_length or len(max_vals) != self.vector_length:
            raise ValueError("All vectors must be of length %d" % self.vector_length)

        for i in range(self.vector_length):
            self.add_variable(name + self.vector_suffixes[i], min_vals[i], max_vals[i])

    def add_variable(self, name, min_val, max_val):
        self.vars.update({name: Variable(name, min_val, max_val)})
        self.dim = len(self.vars)

    def print_num_dimensions(self):
        rospy.loginfo("Your " + self.__class__.__name__ + " has " + str(self.dim) + " dimensions. Variable names are: " + str(self.vars.keys()))

    def get_min_vals(self):
        min_vals = [v.min_val for v in self.vars.values()]
        rospy.loginfo("Min values of " + self.__class__.__name__ + f" are {min_vals}.")
        return np.float32(min_vals)

    def get_max_vals(self):
        max_vals = [v.max_val for v in self.vars.values()]
        rospy.loginfo("Max values of " + self.__class__.__name__ + f" are {max_vals}.")
        return np.float32(max_vals)

    def set_variable(self, name, val):
        self.vars[name].cur_val = np.float32(val)

    def set_variable_from_vector(self, name, xyz_vector):
        for i in range(self.vector_length):
            self.set_variable(name + self.vector_suffixes[i], xyz_vector[i])

    def clip_and_normalize(self, val, min, max, name, to_min=0, to_max=1):
        if np.less(val, min) or np.greater(val, max):
            val = np.clip(val, min, max)
            rospy.logwarn(f"Oops value {val} with name {name} is out of bounds [{min},{max}]! Clipped {name} value is {val}.")
        return np.interp(val, [min, max], [to_min, to_max])

    def get_cur_vals(self, default_val=0, verbose=False):
        with self.mutex:
            # create copy of variables to leave original variables untouched
            vals = deepcopy(self.vars)

            for k, v in vals.items():
                if not np.isnan(v.cur_val):
                    vals[k] = self.clip_and_normalize(v.cur_val, v.min_val, v.max_val, v.name)
                else:
                    vals[k] = default_val

            if verbose:
                rospy.loginfo("Current values are:")
                for k, v in vals.items():
                    rospy.loginfo(f"- {k:20} \t {v}")

            return vals
