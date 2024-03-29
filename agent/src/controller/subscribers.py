import random
from multiprocessing import Lock

import rospy
import numpy as np

from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import Hand
from sensor_listener.msg import ContactFrames
from gazebo_msgs.msg import ContactsState

from .helpers.services import ros_vector_to_list as to_list
from .helpers.transforms import normalize


class Subscribers:
    def __init__(self, hparams, state, obs, gi):

        self.hparams = hparams
        self.state = state
        self.obs = obs
        self.gi = gi

        self.mutex = Lock()
        self.random_gen = random.Random()
        self.random_gen.seed(hparams["seed"])

        rospy.Subscriber("gazebo/object_sensor_bumper", ContactsState, self.object_bumper_callback, queue_size=1)
        rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.ri_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/hand_state", Hand, self.reflex_callback, queue_size=1)

        # only allow contact sensing in force framework 1,2,3
        if self.hparams["force_framework"] in [1, 2, 3]:
            rospy.Subscriber("reflex_takktile/sim_contact_frames", ContactFrames, self.contacts_callback, queue_size=1)

    def object_bumper_callback(self, msg):
        with self.state.mutex:
            for i in range(len(msg.states)):
                for name in ["ground_plane", "sphere_mount"]:
                    if name in msg.states[i].collision1_name or name in msg.states[i].collision2_name:
                        self.state.object_lifted = False
                        return
            self.state.object_lifted = True

    def ri_callback(self, msg):
        with self.state.mutex:
            self.state.num_contacts = msg.num_contacts
            # in theory epsilon force is already in range [0,1] but practically it is rarely larger than 0.7
            self.state.epsilon_force = normalize(msg.epsilon_force, 0, 0.7)
            self.state.epsilon_torque = normalize(msg.epsilon_torque, 0, 0.03)
            self.state.delta_task = self.clip_and_normalize(msg.delta_task, 0, 12)
            self.state.delta_cur = self.clip_and_normalize(msg.delta_cur, 0, 12)
            self.state.sum_contact_forces = msg.sum_contact_forces

    def clip_and_normalize(self, val, low_bound, high_bound):
        val = np.clip(val, low_bound, high_bound)
        return normalize(val, low_bound, high_bound)

    def reflex_callback(self, msg):
        with self.obs.mutex:
            for i in range(self.obs.num_fingers):
                id_str = "_f" + str(i + 1)
                self.obs.set_variable("prox_angle" + id_str, msg.finger[i].proximal)
                self.obs.set_variable("dist_angle" + id_str, msg.finger[i].distal_approx)
                self.set_torque_val("motor_torque" + id_str, msg.motor[i].load)
                self.state.prox_angles[i] = msg.finger[i].proximal

            self.obs.set_variable("preshape_angle", msg.motor[3].joint_angle)
            self.set_torque_val("preshape_motor_torque", msg.motor[3].load)

    def contacts_callback(self, msg):
        with self.obs.mutex:
            self.obs.reset_contact_obs()
            for i in range(msg.num_contact_frames):
                id_str = "_p" + str(msg.contact_frames_shell[i].hand_part_id)
                # palm
                if msg.contact_frames_shell[i].palm_contact:
                    self.obs.set_variable_from_vector("contact_normal" + id_str, to_list(msg.contact_frames_shell[i].contact_normal))
                    self.obs.set_variable_from_vector("contact_pos" + id_str, to_list(msg.contact_frames_shell[i].contact_position))
                    self.set_force_val("contact_force" + id_str, msg.contact_frames_shell[i])
                    continue
                # fingers
                link = "_prox" if msg.contact_frames_shell[i].prox_contact else "_dist"
                self.obs.set_variable_from_vector("contact_normal" + id_str + link, to_list(msg.contact_frames_shell[i].contact_normal))
                self.obs.set_variable_from_vector("contact_pos" + id_str + link, to_list(msg.contact_frames_shell[i].contact_position))
                self.set_force_val("contact_force" + id_str + link, msg.contact_frames_shell[i])

    def set_force_val(self, name, cf_shell):
        if self.hparams["force_framework"] == 1:  # full force vector
            self.obs.set_variable_from_vector(name, [cf_shell.normal_force, cf_shell.tang_force_y, cf_shell.tang_force_z])
        elif self.hparams["force_framework"] == 2:  # only normal force
            self.obs.set_variable(name, cf_shell.normal_force)  # remember: normal is a unit vector, so we can take dot product directly
        elif self.hparams["force_framework"] == 3:  # only binary contact signals
            force = to_list(cf_shell.contact_wrench.force)
            self.obs.set_variable(name, int(np.linalg.norm(force) > 0))
        elif self.hparams["force_framework"] == 4:  # no force info
            return
        else:
            raise ValueError("force_framework must be either 1, 2, 3 or 4")

    def set_torque_val(self, name, val):
        if self.hparams["torque_framework"] == 1:  # perfect torque info
            self.obs.set_variable(name, val)
        elif self.hparams["torque_framework"] == 2:  # noisy torque info
            val += self.random_gen.gauss(0, self.hparams["torque_noise"] * self.obs.motor_torque_max)
            self.obs.set_variable(name, val)
        elif self.hparams["torque_framework"] == 3:  # no torque info
            return
        else:
            raise ValueError("torque_framework must be either 1, 2 or 3")
