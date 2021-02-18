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

    def get_space_low(self):
        space_low = np.empty((0,))
        for i in range(self.dim):
            space_low = np.append(space_low, self.vars[i].min_val)
        print("Min values of " + self.__class__.__name__ + f" are {space_low}.")
        return space_low

    def get_space_high(self):
        space_high = np.empty((0,))
        for i in range(self.dim):
            space_high = np.append(space_high, self.vars[i].max_val)

        print("Max values of " + self.__class__.__name__ + f" is {space_high}.")
        return space_high

    def add_variable(self, var, num_instances):
        self.vars.extend([var for i in range(num_instances)])
        self.dim = len(self.vars)
