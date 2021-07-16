from multiprocessing import Lock
from copy import deepcopy

import numpy as np
import rospy

from ..helpers.transforms import normalize_list_items


class Variable:
    """Defines one action or observation variable."""

    def __init__(self, name, init_value, min_val, max_val):
        self.name = name
        self.cur_val = init_value
        self.min_val = min_val
        self.max_val = max_val


class Space:
    """Parent class for observation and action class."""

    def __init__(self):
        self.vars = []
        self.mutex = Lock()
        self.dim = 0

    def add_variable(self, num_instances, name, init_value, min_val, max_val):
        self.vars.extend([Variable(name, init_value, min_val, max_val) for i in range(num_instances)])
        self.dim = len(self.vars)

    def print_num_dimensions(self):
        rospy.loginfo("Your " + self.__class__.__name__ + " has " + str(self.dim) + " dimensions.")

    def get_min_vals(self):
        min_vals = np.empty((0,))
        for i in range(self.dim):
            min_vals = np.append(min_vals, self.vars[i].min_val)
        rospy.loginfo("Min values of " + self.__class__.__name__ + f" are {min_vals}.")
        return np.float32(min_vals)

    def get_max_vals(self):
        max_vals = np.empty((0,))
        for i in range(self.dim):
            max_vals = np.append(max_vals, self.vars[i].max_val)
        rospy.loginfo("Max values of " + self.__class__.__name__ + f" are {max_vals}.")
        return np.float32(max_vals)

    def set_cur_val_by_name(self, name, cur_val):
        for i in range(self.dim):
            if self.vars[i].name == name:
                self.vars[i].cur_val = cur_val
                return
        raise ValueError(f"Variable with name {name} not found.")

    def clip(self, val, min, max, name):

        if np.any(np.less(val, min)) or np.any(np.greater(val, max)):
            rospy.logwarn(f"Oops value {val} with name {name} is out of bounds [{min},{max}]!")
            val = np.clip(val, min, max)
            rospy.logwarn(f"Clipped {name} value is {val}")
        return val

    def normalize(self, val, from_min, from_max, to_min=0, to_max=1):
        return np.interp(val, [from_min, from_max], [to_min, to_max])

    def get_cur_vals(self, verbose=False, normalize=True):
        with self.mutex:
            vals_dict = {}
            for i in range(self.dim):
                if verbose:
                    rospy.loginfo("Cur value of " + self.vars[i].name + f" is {self.vars[i].cur_val}.")

                val = deepcopy(self.vars[i].cur_val)
                val = np.float32(val)
                default_val = 0

                # we have a scalar
                if type(val) is np.float32:
                    if np.isnan(val):
                        vals_dict.update({self.vars[i].name: default_val})
                        continue
                    val = self.clip(val, self.vars[i].min_val, self.vars[i].max_val, self.vars[i].name)
                    if normalize:
                        val = self.normalize(val, self.vars[i].min_val, self.vars[i].max_val)
                    vals_dict.update({self.vars[i].name: val})

                # we have an array
                else:
                    if np.any(np.isnan(val)):
                        vals_dict.update({f"{self.vars[i].name}_x": default_val, f"{self.vars[i].name}_y": default_val, f"{self.vars[i].name}_z": default_val})
                        continue
                    val = self.clip(val, self.vars[i].min_val, self.vars[i].max_val, self.vars[i].name)
                    for j in range(len(val)):
                        val[j] = self.normalize(val[j], self.vars[i].min_val[j], self.vars[i].max_val[j])
                    vals_dict.update({f"{self.vars[i].name}_x": val[0], f"{self.vars[i].name}_y": val[1], f"{self.vars[i].name}_z": val[2]})

                if verbose:
                    rospy.loginfo("Current values are")
                    for key, value in vals_dict.items():
                        rospy.loginfo(f"- {key:20}{value}")

            return vals_dict

    def get_cur_vals_by_name(self, name, verbose=False):
        "Returns a concatenation of all current values that match the given name."
        with self.mutex:
            cur_vals = np.empty((0,))
            for i in range(self.dim):
                if self.vars[i].name == name:
                    cur_vals = np.append(cur_vals, self.vars[i].cur_val)
            if verbose:
                rospy.loginfo("Cur values of " + self.__class__.__name__ + f" that match the name {name} are {cur_vals}.")
            return np.float32(cur_vals)

    def print_all_cur_vals(self):
        with self.mutex:
            rospy.loginfo("\n=== Cur values of " + self.__class__.__name__ + f" ===")
            for i in range(self.dim):
                rospy.loginfo(self.vars[i].name + ": " + str(self.vars[i].cur_val))

    def print_min_vals(self):
        rospy.loginfo("\n=== Min values of " + self.__class__.__name__ + f" ===")
        for i in range(self.dim):
            rospy.loginfo(self.vars[i].name + ": " + str(self.vars[i].min_val))

    def print_max_vals(self):
        rospy.loginfo("\n=== Max values of " + self.__class__.__name__ + f" ===")
        for i in range(self.dim):
            rospy.loginfo(self.vars[i].name + ": " + str(self.vars[i].max_val))
