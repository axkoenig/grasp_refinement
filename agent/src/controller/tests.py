import random
import math
import csv
import pickle
import os

import numpy as np

from .helpers.transforms import is_valid_vertical_offset, deg2rad

controller_dir = os.path.dirname(os.path.realpath(__file__))
TEST_CASES_PKL = os.path.join(controller_dir, "test_cases/test_cases.pkl")
TEST_CASES_CSV = os.path.join(controller_dir, "test_cases/test_cases.csv")


def sample_from_range(range):
    return np.random.uniform(min(range), max(range))


def sample_sign():
    return random.choice([-1, 1])


def complete_vec_to_length(var1, var2, length):
    return math.sqrt(pow(length, 2) - pow(var1, 2) - pow(var2, 2))


def get_vector_with_length(length, x_error_min, x_error_max, y_error_min, y_error_max, z_error_min, z_error_max):
    while True:
        # sample two values from range and get other one from pythagoras
        x = np.clip(sample_from_range([0, sample_sign() * length]), x_error_min, x_error_max)
        z = np.clip(sample_from_range([0, sample_sign() * length]), z_error_min, z_error_max)
        try:
            y = sample_sign() * complete_vec_to_length(x, z, length)
            if y < y_error_min or y > y_error_max:
                continue
            return [x, y, z]
        except ValueError:  # this combination doesn't work
            continue


def gen_object(object_type):
    if object_type == "sphere":
        return RandomSphere()
    elif object_type == "cylinder":
        return RandomCylinder()
    elif object_type == "box":
        return RandomBox()
    else:
        raise ValueError("Unsupported object name.")


def gen_valid_wrist_error_from_l2(object, trans_l2_error, rot_l2_error, hparams):
    wrist_error = RandomWristErrorFromL2(trans_l2_error, rot_l2_error, hparams)
    # if this wrist_error, object combination would crash hand into ground, we re-generate a new one
    while not is_valid_vertical_offset(wrist_error.y, object.get_height()):
        wrist_error = RandomWristErrorFromL2(trans_l2_error, rot_l2_error, hparams)
    return wrist_error


def gen_valid_wrist_error_obj_combination_from_ranges(object_type, hparams):
    object = gen_object(object_type)

    x_range = [hparams["x_error_min"], hparams["x_error_max"]]
    y_range = [hparams["y_error_min"], hparams["y_error_max"]]
    z_range = [hparams["z_error_min"], hparams["z_error_max"]]
    roll_range = [hparams["roll_error_min"], hparams["roll_error_max"]]
    pitch_range = [hparams["pitch_error_min"], hparams["pitch_error_max"]]
    yaw_range = [hparams["yaw_error_min"], hparams["yaw_error_max"]]

    wrist_error = RandomWristErrorFromRanges(x_range, y_range, z_range, roll_range, pitch_range, yaw_range)

    # if this (wrist_error, object) combination would crash hand into ground, we re-generate a new one
    while not is_valid_vertical_offset(wrist_error.y, object.get_height()):
        wrist_error = RandomWristErrorFromRanges(x_range, y_range, z_range, roll_range, pitch_range, yaw_range)
        object = gen_object(object_type)
    return object, wrist_error


class Sphere:
    def __init__(self, radius, mass, inertia_scaling_factor):
        self.radius = radius
        self.mass = mass
        self.inertia_scaling_factor = inertia_scaling_factor
        self.name = "sphere_" + str(self.radius)
        self.type = "sphere"

    def get_csv_data(self):
        return dict({"dimension_1": self.radius, "dimension_2": 0, "dimension_3": 0, "mass": self.mass, "inertia_scaling_factor": self.inertia_scaling_factor})

    def get_height(self):
        return self.radius * 2


class Cylinder:
    def __init__(self, radius, length, mass, inertia_scaling_factor):
        self.radius = radius
        self.length = length
        self.mass = mass
        self.inertia_scaling_factor = inertia_scaling_factor
        self.name = "cylinder_" + str(self.radius) + "_" + str(self.length)
        self.type = "cylinder"

    def get_csv_data(self):
        return dict(
            {"dimension_1": self.radius, "dimension_2": self.length, "dimension_3": 0, "mass": self.mass, "inertia_scaling_factor": self.inertia_scaling_factor}
        )

    def get_height(self):
        return self.length


class Box:
    def __init__(self, x, y, z, mass, inertia_scaling_factor):
        self.x = x
        self.y = y
        self.z = z
        self.mass = mass
        self.inertia_scaling_factor = inertia_scaling_factor
        self.name = "box_" + str(self.x) + "_" + str(self.y) + "_" + str(self.z)
        self.type = "box"

    def get_csv_data(self):
        return dict(
            {"dimension_1": self.x, "dimension_2": self.y, "dimension_3": self.z, "mass": self.mass, "inertia_scaling_factor": self.inertia_scaling_factor}
        )

    def get_height(self):
        return self.z


class RandomSphere(Sphere):
    def __init__(self, radius_range=[0.065, 0.08], mass_range=[0.1, 0.4], inertia_scaling_factor=10):
        super().__init__(sample_from_range(radius_range), sample_from_range(mass_range), inertia_scaling_factor)


class RandomCylinder(Cylinder):
    def __init__(self, radius_range=[0.03, 0.05], length_range=[0.13, 0.23], mass_range=[0.1, 0.4], inertia_scaling_factor=10):
        super().__init__(sample_from_range(radius_range), sample_from_range(length_range), sample_from_range(mass_range), inertia_scaling_factor)


class RandomBox(Box):
    def __init__(self, x_range=[0.04, 0.10], y_range=[0.04, 0.10], z_range=[0.13, 0.23], mass_range=[0.1, 0.4], inertia_scaling_factor=10):
        super().__init__(
            sample_from_range(x_range), sample_from_range(y_range), sample_from_range(z_range), sample_from_range(mass_range), inertia_scaling_factor
        )


class RandomWristError:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0


class RandomWristErrorFromL2(RandomWristError):
    def __init__(self, trans_l2_error, rot_l2_error, hparams):
        super().__init__()
        result = get_vector_with_length(
            trans_l2_error,
            hparams["x_error_min"],
            hparams["x_error_max"],
            hparams["y_error_min"],
            hparams["y_error_max"],
            hparams["z_error_min"],
            hparams["z_error_max"],
        )
        self.x = result[0]
        self.y = result[1]
        self.z = result[2]
        result = get_vector_with_length(
            rot_l2_error,
            hparams["roll_error_min"],
            hparams["roll_error_max"],
            hparams["pitch_error_min"],
            hparams["pitch_error_max"],
            hparams["yaw_error_min"],
            hparams["yaw_error_max"],
        )
        self.roll = result[0]
        self.pitch = result[1]
        self.yaw = result[2]


class RandomWristErrorFromRanges(RandomWristError):
    def __init__(self, x_range, y_range, z_range, roll_range, pitch_range, yaw_range):
        super().__init__()
        self.x = sample_from_range(x_range)
        self.y = sample_from_range(y_range)
        self.z = sample_from_range(z_range)
        self.roll = sample_from_range(roll_range)
        self.pitch = sample_from_range(pitch_range)
        self.yaw = sample_from_range(yaw_range)


class TestCase:
    def __init__(self, hparams):
        # default parameters (will be overriden by child classes)
        self.object = RandomSphere()
        self.trans_l2_error = 0
        self.rot_l2_error = 0
        self.wrist_error = RandomWristErrorFromL2(0, 0, hparams)

    def get_csv_data(self):
        data = dict(
            {
                "object_type": self.object.type,
                "trans_l2_error": self.trans_l2_error,
                "rot_l2_error": self.rot_l2_error,
                "wrist_x": self.wrist_error.x,
                "wrist_y": self.wrist_error.y,
                "wrist_z": self.wrist_error.z,
                "wrist_roll": self.wrist_error.roll,
                "wrist_pitch": self.wrist_error.pitch,
                "wrist_yaw": self.wrist_error.yaw,
            }
        )
        data.update(self.object.get_csv_data())
        return data

    def get_csv_header(self):
        d = self.get_csv_data()
        return [*d]


class TestCaseFromObjectAndL2Errors(TestCase):
    def __init__(self, object, trans_l2_error, rot_l2_error, hparams):
        # for this test case we know the object and want a suitable l2 wrist error
        super().__init__(hparams)
        self.object = object
        self.trans_l2_error = trans_l2_error
        self.rot_l2_error = rot_l2_error
        self.wrist_error = gen_valid_wrist_error_from_l2(self.object, self.trans_l2_error, self.rot_l2_error, hparams)


class TestCaseFromRanges(TestCase):
    def __init__(self, hparams):
        # for this case we only know the ranges of object size and wrist error
        super().__init__(hparams)
        object_type = random.choice(["sphere", "cylinder", "box"])
        self.object, self.wrist_error = gen_valid_wrist_error_obj_combination_from_ranges(object_type, hparams)
        self.trans_l2_error = np.linalg.norm([self.wrist_error.x, self.wrist_error.y, self.wrist_error.z])
        self.rot_l2_error = np.linalg.norm([self.wrist_error.roll, self.wrist_error.pitch, self.wrist_error.yaw])


class TestCases:
    def __init__(self, hparams, num_exp_per_obj=10):
        self.test_cases = []
        object_types = ["sphere", "cylinder", "box"]
        self.trans_l2_errors = np.arange(0, 8) / 100  # 0 to 7 cm
        self.rot_l2_errors = np.arange(0, 8) * 2  # 0, 2, ... , 14 deg
        for object_type in object_types:
            for _ in range(num_exp_per_obj):
                object = gen_object(object_type)
                for i in range(len(self.trans_l2_errors)):
                    self.test_cases.append(TestCaseFromObjectAndL2Errors(object, self.trans_l2_errors[i], deg2rad(self.rot_l2_errors[i]), hparams))


def generate_test_cases(hparams):
    # generate test cases and save to disk
    t = TestCases(hparams)
    with open(TEST_CASES_PKL, "wb") as file:
        pickle.dump(t, file)

    # also save csv to disk
    with open(TEST_CASES_CSV, "w") as file:
        writer = csv.DictWriter(file, fieldnames=t.test_cases[0].get_csv_header())
        writer.writeheader()
        for case in t.test_cases:
            writer.writerows([case.get_csv_data()])


def test(model, env, log_path, log_name, deterministic=False, all_test_cases=True, trans_l2_errors=[0.06, 0.07]):
    # load test cases from disk
    with open(TEST_CASES_PKL, "rb") as file:
        t = pickle.load(file)

    metrics = ["sustained_lifting", "sustained_holding"]

    # create output csv file
    path = os.path.join(log_path, log_name + ".csv")
    with open(path, "w") as file:
        fieldnames = t.test_cases[0].get_csv_header()
        for metric in metrics:
            fieldnames.append(metric)
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        # running on subset of test cases: remove the boring test cases and shuffle
        if not all_test_cases:
            t.test_cases = [x for x in t.test_cases if x.trans_l2_error in trans_l2_errors]
            random.shuffle(t.test_cases)

        for test_case in t.test_cases:
            obs = env.reset(test_case)
            while True:
                action, state = model.predict(obs, deterministic=deterministic)
                obs, reward, done, info = env.step(action)
                if done:
                    # save experiment outcome
                    data = test_case.get_csv_data()
                    outcome = {metric: info[metric] for metric in metrics}
                    data.update(outcome)
                    writer.writerows([data])
                    file.flush()
                    break
