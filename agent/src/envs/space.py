import numpy as np


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
        self.dim = 0

    def add_variable(self, num_instances, name, init_value, min_val, max_val):
        self.vars.extend([Variable(name, init_value, min_val, max_val) for i in range(num_instances)])
        self.dim = len(self.vars)

    def get_min_vals(self):
        min_vals = np.empty((0,))
        for i in range(self.dim):
            min_vals = np.append(min_vals, self.vars[i].min_val)
        print("Min values of " + self.__class__.__name__ + f" are {min_vals}.")
        return min_vals

    def get_max_vals(self):
        max_vals = np.empty((0,))
        for i in range(self.dim):
            max_vals = np.append(max_vals, self.vars[i].max_val)
        print("Max values of " + self.__class__.__name__ + f" are {max_vals}.")
        return max_vals

    def get_cur_vals(self, verbose=False):
        cur_vals = np.empty((0,))
        for i in range(self.dim):
            if verbose:
                print("Cur value of " + self.vars[i].name + f" is {self.vars[i].cur_val}.")
            cur_vals = np.append(cur_vals, self.vars[i].cur_val)
        return cur_vals

    def get_cur_vals_by_name(self, name, verbose=False):
        "Returns a concatenation of all curent values that match the given name."
        cur_vals = np.empty((0,))
        for i in range(self.dim):
            if self.vars[i].name == name:
                cur_vals = np.append(cur_vals, self.vars[i].cur_val)
        if verbose:
            print("Cur values of " + self.__class__.__name__ + f" that match the name {name} are {cur_vals}.")
        return cur_vals

    def set_cur_val_by_name(self, name, cur_val):
        for i in range(self.dim):
            if self.vars[i].name == name:
                self.vars[i].cur_val = cur_val
                return
        raise ValueError("Variable with name {name} not found.")

    def print_all_cur_vals(self):
        print("\n=== Cur values of " + self.__class__.__name__ + f" ===")
        for i in range(self.dim):
            print(self.vars[i].name + ": " + str(self.vars[i].cur_val))
