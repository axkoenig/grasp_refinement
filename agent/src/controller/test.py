import rospy
import numpy as np


def sample_from_range(range):
    return np.random.uniform(range[0], range[1])


class Cylinder:
    def __init__(self, radius, length, inertia_scaling_factor):
        self.radius = radius
        self.length = length
        self.inertia_scaling_factor = inertia_scaling_factor
        self.name = "cylinder_" + str(self.radius) + "_" + str(self.length)


class Box:
    def __init__(self, x, y, z, inertia_scaling_factor):
        self.x = x
        self.y = y
        self.z = z
        self.inertia_scaling_factor = inertia_scaling_factor
        self.name = "box_" + str(self.x) + "_" + str(self.y) + "_" + str(self.z)


class RandomCylinder(Cylinder):
    def __init__(self, radius_range=[0.03, 0.05], length_range=[0.1, 0.2], inertia_scaling_factor=0.9):
        super().__init__(sample_from_range(radius_range), sample_from_range(length_range), inertia_scaling_factor)


class RandomBox(Box):
    def __init__(self, x_range=[0.04, 0.10], y_range=[0.04, 0.10], z_range=[0.1, 0.2], inertia_scaling_factor=0.9):
        super().__init__(sample_from_range(x_range), sample_from_range(y_range), sample_from_range(z_range), inertia_scaling_factor)


# class TestCase:
#     def __init__(self, object_name):
#         self.object_name = object_name
#         self.object = RandomCylinder() if object_name == "cylinder" else RandomBox()


# def generate_test_cases():
#     objects = ["box", "cylinder"]


# def test(model, env, logger, num_exp_per_obj=10):
#     rospy.loginfo("Testing model ...")

#     for object in objects:


# def test_episode(model, env, logger):
#     obs = env.reset()
#     while True:
#         action, _state = model.predict(obs, deterministic=True)
#         obs, reward, done, info = env.step(action)
#         if done:
#             break