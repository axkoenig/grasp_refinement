from multiprocessing import Lock

import rospy

from .stage import Stage


class State:
    def __init__(self):
        self.mutex = Lock()

        # this info comes from reflex interface
        self.epsilon_force = 0
        self.epsilon_torque = 0
        self.delta_task = 0
        self.delta_cur = 0
        self.num_contacts = 0
        self.sum_contact_forces = 0
        self.prox_angles = [0, 0, 0]

        # this info is modified by Controller
        self.stage = Stage(0)
        self.obj_shift = 0
        self.dist_tcp_obj = 0
        self.num_regrasps = 0
        self.cur_time_step = 0
        self.cur_test_case = None
        self.object_lifted = False
        self.sustained_holding = False
        self.sustained_lifting = False
        self.last_time_stamp = rospy.Time.now()
        self.io_buffer = []  # array of dicts

    def reset(self):
        # only reset some variables as the other ones will be continously updated anyway
        self.stage = Stage.REFINE
        self.num_regrasps = 0
        self.cur_time_step = 0
        self.sustained_holding = False
        self.sustained_lifting = False
        self.last_time_stamp = rospy.Time.now()
        self.io_buffer = []

    def store_io_in_buffer(self, obs_dict, action_dict, reward, infos_dict):

        # we only log quality metrics for now
        desired_infos = ["a_epsilon_force", "a_epsilon_torque", "a_delta_task", "a_delta_cur"]
        infos_dict = {key: value for key, value in infos_dict.items() if key in desired_infos}

        # concat into one dict
        super_dict = {}
        dicts = [self.cur_test_case.get_csv_data(), obs_dict, action_dict, {"reward": reward}, infos_dict]
        for d in dicts:
            super_dict.update(d)

        self.io_buffer.append(super_dict)
