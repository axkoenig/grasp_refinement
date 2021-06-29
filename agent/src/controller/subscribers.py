import random

import rospy
import numpy as np

from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import Hand
from sensor_listener.msg import ContactFrames
from gazebo_msgs.msg import ContactsState

from .helpers.services import ros_vector_to_list as to_list


class Subscribers:
    def __init__(self, hparams, state, obs, gi):

        self.hparams = hparams
        self.state = state
        self.obs = obs
        self.gi = gi

        self.random_gen = random.Random()
        self.random_gen.seed(hparams["seed"])

        rospy.Subscriber("gazebo/object_sensor_bumper", ContactsState, self.object_bumper_callback, queue_size=1)
        rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.ri_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/hand_state", Hand, self.reflex_callback, queue_size=1)

        # we don't have any contact sensing in force framework 4
        if self.hparams["force_sensing"] != 4:
            rospy.Subscriber("reflex_takktile/sim_contact_frames", ContactFrames, self.contacts_callback, queue_size=1)

    def object_bumper_callback(self, msg):
        with self.state.mutex:
            ground = "ground_plane"
            for i in range(len(msg.states)):
                if ground in msg.states[i].collision1_name or ground in msg.states[i].collision2_name:
                    self.state.object_lifted = False
                    return
            self.state.object_lifted = True

    def ri_callback(self, msg):
        with self.state.mutex:
            self.state.num_contacts = msg.num_contacts
            self.state.epsilon = self.clip_and_normalize(msg.epsilon, 0, 0.03)
            self.state.delta_task = self.clip_and_normalize(msg.delta_task, -5, 12)
            self.state.delta_cur = self.clip_and_normalize(msg.delta_cur, -5, 12)
            self.state.sum_contact_forces = msg.sum_contact_forces

    def normalize(self, val, low_bound, high_bound):
        return (val - low_bound) / (high_bound - low_bound)

    def clip_and_normalize(self, val, low_bound, high_bound):
        if not val:
            return 0
        val = np.clip(val, low_bound, high_bound)
        return self.normalize(val, low_bound, high_bound)

    def reflex_callback(self, msg):
        with self.obs.mutex:
            for i in range(self.obs.num_fingers):
                id_str = "_f" + str(i + 1)
                self.obs.set_cur_val_by_name("prox_angle" + id_str, msg.finger[i].proximal)
                self.obs.set_cur_val_by_name("dist_angle" + id_str, msg.finger[i].distal_approx)
                self.set_torque_val("motor_torque" + id_str, msg.motor[i].load)
                self.state.prox_angles[i] = msg.finger[i].proximal

            self.obs.set_cur_val_by_name("preshape_angle", msg.motor[3].joint_angle)
            self.set_torque_val("preshape_motor_torque", msg.motor[3].load)

    def contacts_callback(self, msg):
        with self.obs.mutex:
            self.obs.reset_contact_obs()
            for i in range(msg.num_contact_frames):
                id_str = "_p" + str(msg.contact_frames_shell[i].hand_part_id)
                # palm
                if msg.contact_frames_shell[i].palm_contact:
                    self.obs.set_cur_val_by_name("contact_normal" + id_str, to_list(msg.contact_frames_shell[i].contact_normal))
                    self.obs.set_cur_val_by_name("contact_pos" + id_str, to_list(msg.contact_frames_shell[i].contact_position))
                    self.set_force_val("contact_force" + id_str, msg.contact_frames_shell[i])
                    continue
                # fingers
                link = "_prox" if msg.contact_frames_shell[i].prox_contact else "_dist"
                self.obs.set_cur_val_by_name("contact_normal" + id_str + link, to_list(msg.contact_frames_shell[i].contact_normal))
                self.obs.set_cur_val_by_name("contact_pos" + id_str + link, to_list(msg.contact_frames_shell[i].contact_position))
                self.set_force_val("contact_force" + id_str + link, msg.contact_frames_shell[i])

    def set_force_val(self, name, cf_shell):
        if self.hparams["force_sensing"] == 1:  # full force vector
            self.obs.set_cur_val_by_name(name, to_list(cf_shell.contact_wrench.force))
        elif self.hparams["force_sensing"] == 2:  # only normal force
            force = to_list(cf_shell.contact_wrench.force)
            normal = to_list(cf_shell.contact_normal)
            self.obs.set_cur_val_by_name(name, abs(np.dot(force, normal)))
        elif self.hparams["force_sensing"] == 3:  # only binary contact signals
            force = to_list(cf_shell.contact_wrench.force)
            self.obs.set_cur_val_by_name(name, int(np.linalg.norm(force) > 0))
        else:
            raise ValueError("Unsupported force sensing framework.")

    def set_torque_val(self, name, val):
        if self.hparams["noisy_torque"]:  # provide noisy torque info (10% of max torque)
            val += self.random_gen.gauss(0, 0.1 * self.obs.motor_torque_max)
        self.obs.set_cur_val_by_name(name, val)
