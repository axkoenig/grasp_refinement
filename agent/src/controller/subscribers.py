import rospy
import numpy as np

from reflex_interface.msg import HandStateStamped
from reflex_msgs.msg import Hand
from sensor_listener.msg import ContactFrames

from .helpers.services import ros_vector_to_list


class Subscribers:
    def __init__(self, state, obs):

        self.state = state
        self.obs = obs

        rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.ri_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/hand_state", Hand, self.reflex_callback, queue_size=1)
        rospy.Subscriber("reflex_takktile/sim_contact_frames", ContactFrames, self.cf_callback, queue_size=1)

    def ri_callback(self, msg):
        self.state.num_contacts = msg.num_contacts
        self.state.epsilon_force = msg.epsilon_force
        self.state.epsilon_torque = msg.epsilon_torque
        self.state.delta_task = np.clip(msg.delta_task, -5, 5)
        self.state.delta_cur = np.clip(msg.delta_cur, -2, 8)
        self.state.sum_contact_forces = msg.sum_contact_forces

        self.obs.set_cur_val_by_name("preshape_angle", msg.preshape_angle)

        for i in range(self.obs.num_fingers):
            # joint positions
            id_str = "_f" + str(i + 1)
            self.state.prox_angles[i] = msg.finger_state[i].proximal_angle
            self.obs.set_cur_val_by_name("prox_angle" + id_str, msg.finger_state[i].proximal_angle)
            self.obs.set_cur_val_by_name("dist_angle" + id_str, msg.finger_state[i].distal_angle)

            # normals
            normal = ros_vector_to_list(msg.finger_state[i].prox_normal)
            self.obs.set_cur_val_by_name("contact_normal_prox" + id_str, normal)
            normal = ros_vector_to_list(msg.finger_state[i].dist_normal)
            self.obs.set_cur_val_by_name("contact_normal_dist" + id_str, normal)

            tactile_positions = np.empty([0, 3])
            pressures = np.empty([0, 1])

            # tactile feedback
            # for j in range(self.obs.num_sensors):
            #     id_str = "_f" + str(i + 1) + "_s" + str(j + 1)
            #     self.obs.set_cur_val_by_name("sensor_pressure" + id_str, msg.finger_state[i].sensor_pressure[j])
            #     self.obs.set_cur_val_by_name("tactile_contact" + id_str, msg.finger_state[i].sensor_contact[j])

            #     # save contact location and pressure if we have a contact
            #     if msg.finger_state[i].sensor_pressure[j] > 0:
            #         tactile_positions = np.append(tactile_positions, [self.gi.ros_vector_to_list(msg.finger_state[i].tactile_position[j])], axis=0)
            #         pressures = np.append(pressures, msg.finger_state[i].sensor_pressure[j])

            #     if j == 4:
            #         # record weighted proximal contact position and reset variables
            #         self.record_contact_pos(tactile_positions, pressures, "tactile_position_f" + str(i + 1) + "_prox")
            #         tactile_positions = np.empty([0, 3])
            #         pressures = np.empty([0, 1])
            #     elif j == 8:
            #         # record weighted distal contact position
            #         self.record_contact_pos(tactile_positions, pressures, "tactile_position_f" + str(i + 1) + "_dist")

    def record_contact_pos(self, tactile_positions, pressures, param_name):
        if pressures.size == 0:  # use default value if no contact on link
            pos = self.obs.tactile_pos_default
        elif pressures.size == 1:  # we only have one contact on link
            pos = tactile_positions
        else:  # multiple contacts on one link, compute weighted average
            pos = np.array([0, 0, 0], dtype=np.float64)
            for i in range(pressures.size):
                pos += pressures[i] * tactile_positions[i]
            pos /= pressures.sum()
            # rospy.logwarn(f"You have {pressures.size} contacts on one link. Computed contact location as weighted average of positions {tactile_positions} and pressures {pressures} as: {pos}")
        self.obs.set_cur_val_by_name(param_name, pos)

    def reflex_callback(self, msg):
        pass

    def cf_callback(self, msg):
        pass
