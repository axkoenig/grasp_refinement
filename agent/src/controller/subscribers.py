from multiprocessing import Lock

import rospy
import numpy as np

from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import Hand
from sensor_listener.msg import ContactFrames

from .helpers.services import ros_vector_to_list as to_list


class Subscribers:
    def __init__(self, state, obs, gi):

        self.state = state
        self.obs = obs
        self.gi = gi
        self.mutex = Lock()

        # sleep here to give node some time to register
        rospy.sleep(1)

        rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.ri_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/hand_state", Hand, self.reflex_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/sim_contact_frames", ContactFrames, self.contacts_callback, queue_size=1)

    def ri_callback(self, msg):
        self.state.num_contacts = msg.num_contacts
        # in theory epsilon force is already in range [0,1] but practically it is rarely larger than 0.7
        self.state.epsilon_force = self.normalize(msg.epsilon_force, 0, 0.7)
        self.state.epsilon_torque = self.normalize(msg.epsilon_torque, 0, 0.03)
        self.state.delta_task = self.clip_and_normalize(msg.delta_task, -5, 10)
        self.state.delta_cur = self.clip_and_normalize(msg.delta_cur, -5, 10)
        self.state.sum_contact_forces = msg.sum_contact_forces

    def normalize(self, val, low_bound, high_bound):
        return (val - low_bound) / (high_bound - low_bound)

    def clip_and_normalize(self, val, low_bound, high_bound):
        val = np.clip(val, low_bound, high_bound)
        return self.normalize(val, low_bound, high_bound)

    def reflex_callback(self, msg):
        with self.mutex:
            for i in range(self.obs.num_fingers):
                id_str = "_f" + str(i + 1)
                self.obs.set_cur_val_by_name("prox_angle" + id_str, msg.finger[i].proximal)
                self.obs.set_cur_val_by_name("dist_angle" + id_str, msg.finger[i].distal_approx)
                self.obs.set_cur_val_by_name("motor_torque" + id_str, msg.motor[i].load)

                self.state.prox_angles[i] = msg.finger[i].proximal

            self.obs.set_cur_val_by_name("preshape_angle", msg.motor[3].joint_angle)
            self.obs.set_cur_val_by_name("preshape_motor_torque", msg.motor[3].load)

    def contacts_callback(self, msg):
        with self.mutex:
            self.obs.reset_contact_obs()
            for i in range(msg.num_contact_frames):
                id_str = "_p" + str(msg.contact_frames_shell[i].hand_part_id)
                # palm
                if msg.contact_frames_shell[i].palm_contact:
                    self.obs.set_cur_val_by_name("contact_normal" + id_str, to_list(msg.contact_frames_shell[i].contact_normal))
                    self.obs.set_cur_val_by_name("contact_pos" + id_str, to_list(msg.contact_frames_shell[i].contact_position))
                    self.obs.set_cur_val_by_name("contact_force" + id_str, to_list(msg.contact_frames_shell[i].contact_wrench.force))
                    continue
                # fingers
                link = "_prox" if msg.contact_frames_shell[i].prox_contact else "_dist"
                self.obs.set_cur_val_by_name("contact_normal" + id_str + link, to_list(msg.contact_frames_shell[i].contact_normal))
                self.obs.set_cur_val_by_name("contact_pos" + id_str + link, to_list(msg.contact_frames_shell[i].contact_position))
                self.obs.set_cur_val_by_name("contact_force" + id_str + link, to_list(msg.contact_frames_shell[i].contact_wrench.force))
