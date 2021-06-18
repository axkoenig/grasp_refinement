import rospy

from .stage import Stage

class State:
    def __init__(self):
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
        self.sustained_holding = False
        self.sustained_lifting = False
        self.last_time_stamp = rospy.Time.now()

    def reset(self):
        # only reset some variables as the other ones will be continously updated anyway
        self.stage = Stage.REFINE
        self.num_regrasps = 0
        self.cur_time_step = 0
        self.sustained_holding = False
        self.sustained_lifting = False
        self.last_time_stamp = rospy.Time.now()
