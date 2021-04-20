import random

import numpy as np
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty, Trigger, TriggerRequest
from reflex_interface.srv import GraspPrimitive
from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState, DeleteModel, SpawnModel, DeleteModelRequest, SpawnModelRequest
from gazebo_msgs.msg import ModelState, ContactsState
from reflex_interface.srv import PosIncrement

from .helpers import get_tq_from_homo_matrix, get_homo_matrix_from_msg, get_homo_matrix_from_tq


class GazeboInterface:
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.set_model_state_name = "/gazebo/set_model_state"
        self.delete_model_name = "/gazebo/delete_model"
        self.spawn_model_name = "/gazebo/spawn_sdf_model"

        self.object_name = rospy.get_param("/object_name")
        self.object_names = rospy.get_param("/object_names")

        # gazebo services
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)
        self.set_model_state = rospy.ServiceProxy(self.set_model_state_name, SetModelState)
        self.delete_model = rospy.ServiceProxy(self.delete_model_name, DeleteModel)
        self.spawn_model = rospy.ServiceProxy(self.spawn_model_name, SpawnModel)
        self.num_srv_tries = 0

        # reflex services
        self.open_hand = rospy.ServiceProxy("/reflex_interface/open", GraspPrimitive)
        self.close_until_contact = rospy.ServiceProxy("/reflex_interface/close_until_contact", Trigger)
        self.pos_incr = rospy.ServiceProxy("/reflex_interface/position_increment", PosIncrement)
        self.srv_time_out = 6
        self.srv_tolerance = 0.1

        # setup listener to measured wrist and object poses
        self.mes_wrist_name = "reflex_interface/wrist_measured"
        self.mes_wrist_buf = tf2_ros.Buffer()
        self.mes_wrist_listener = tf2_ros.TransformListener(self.mes_wrist_buf)
        self.mes_obj_name = "reflex_interface/obj_measured"
        self.mes_obj_buf = tf2_ros.Buffer()
        self.mes_obj_listener = tf2_ros.TransformListener(self.mes_obj_buf)

        # setup broadcaster for desired wrist pose
        self.ts_wrist = geometry_msgs.msg.TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.sim_unpause()
        rospy.sleep(1)  # broadcaster needs some time to start

    def sim_unpause(self):
        self.service_call_with_retries(self.unpause, None, self.unpause_name)

    def sim_pause(self):
        self.service_call_with_retries(self.pause, None, self.pause_name)

    def ros_vector_to_list(self, ros_vector):
        return [ros_vector.x, ros_vector.y, ros_vector.z]

    def run_for_seconds(self, secs, print_string=""):
        self.sim_unpause()
        if len(print_string) > 0:
            rospy.loginfo(print_string)
        rospy.sleep(secs)
        self.sim_pause()

    def get_object_pose(self):
        return self.get_measured_pose(self.mes_obj_buf, self.mes_obj_name)

    def get_wrist_pose(self):
        return self.get_measured_pose(self.mes_wrist_buf, self.mes_wrist_name)

    def get_measured_pose(self, buffer, name, frame="world"):
        try:
            trans = buffer.lookup_transform(frame, name, rospy.Time())
            return get_homo_matrix_from_msg(trans.transform, name, frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo("Couldn't look up object pose. Exception: " + str(e))

    def get_trans_tcp_obj(self):
        mat_shell = self.get_wrist_pose()
        mat_obj = self.get_object_pose()

        # add tcp offset to current shell transform
        mat_tcp = tf.transformations.translation_matrix([0.02, 0, 0.09228])
        mat_tcp = np.dot(mat_shell, mat_tcp)

        # get transform from tcp to object
        mat_tcp_to_obj = np.dot(tf.transformations.inverse_matrix(mat_tcp), mat_obj)
        return tf.transformations.translation_from_matrix(mat_tcp_to_obj)

    def get_dist_tcp_obj(self):
        return np.linalg.norm(self.get_trans_tcp_obj())

    def service_call(self, service, request, service_name):
        rospy.wait_for_service(service_name)
        try:
            service(request) if request else service()
            return True, f"Service call to {service_name} succeeded."
        except rospy.ServiceException as e:
            return False, f"Service call to {service_name} failed with exception: {str(e)}"

    def service_call_with_retries(self, service, request, service_name, max_retries=10):
        tries = 0
        while tries < max_retries:
            success, msg = self.service_call(service, request, service_name)
            if success:
                return
            tries += 1
        rospy.loginfo(f"Service call to {service_name} failed even after {max_retries}. Exception was: {msg}.")

    def set_model_pose(self, pose, model_name, reference_frame="world"):
        t, q = get_tq_from_homo_matrix(pose)
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = reference_frame
        state.pose.position.x = t[0]
        state.pose.position.y = t[1]
        state.pose.position.z = t[2]
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        self.service_call_with_retries(self.set_model_state, state, self.set_model_state_name)

    def send_transform(self, pose):
        t, q = get_tq_from_homo_matrix(pose)
        self.ts_wrist.header.stamp = rospy.Time.now()
        self.ts_wrist.header.frame_id = "world"
        self.ts_wrist.child_frame_id = "reflex"
        self.ts_wrist.transform.translation.x = t[0]
        self.ts_wrist.transform.translation.y = t[1]
        self.ts_wrist.transform.translation.z = t[2]
        self.ts_wrist.transform.rotation.x = q[0]
        self.ts_wrist.transform.rotation.y = q[1]
        self.ts_wrist.transform.rotation.z = q[2]
        self.ts_wrist.transform.rotation.w = q[3]
        self.br.sendTransform(self.ts_wrist)

    def cmd_wrist_abs(self, mat_shell, wait_until_reached_pose=False):
        self.last_wrist_pose = mat_shell
        self.send_transform(mat_shell)
        if wait_until_reached_pose:
            self.wait_until_reached_pose(mat_shell)

    def cmd_wrist_pose_incr(self, p_incr, q_incr, wait_until_reached_pose=False):
        mat_t_incr = tf.transformations.translation_matrix(p_incr)
        mat_q_incr = tf.transformations.quaternion_matrix(q_incr)
        mat_homo = np.dot(mat_t_incr, mat_q_incr)
        self.last_wrist_pose = np.dot(self.last_wrist_pose, mat_homo)
        self.send_transform(self.last_wrist_pose)
        if wait_until_reached_pose:
            self.wait_until_reached_pose(self.last_wrist_pose)

    def wait_until_reached_pose(self, pose, t_tol=0.005, q_tol=0.02):
        r = rospy.Rate(5)
        t, q = get_tq_from_homo_matrix(pose)
        while not rospy.is_shutdown():
            mat_shell = self.get_wrist_pose()
            t_cur, q_cur = get_tq_from_homo_matrix(mat_shell)
            if np.linalg.norm(t_cur - t) > t_tol and self.verbose:
                rospy.loginfo(f"Wrist position not within tolerance of {t_tol} yet.")
            elif np.linalg.norm(q_cur - q) > t_tol and self.verbose:
                rospy.loginfo(f"Wrist orientation not within tolerance of {q_tol} yet.")
            else:
                return
            r.sleep()

    def wait_until_grasp_stabilizes(self):
        rospy.sleep(0.2)

    def delete_object(self):
        req = DeleteModelRequest()
        req.model_name = "object"
        self.delete_model(req)
        self.service_call_with_retries(self.delete_model, req, self.delete_model_name)

    def reset_world(self, pos_error):

        self.delete_object()
        self.sim_unpause()

        # open fingers and move wrist to start position
        self.open_hand(True, self.srv_tolerance, self.srv_time_out)
        self.select_random_object_wrist_pair()
        wrist_init_pose = self.get_wrist_init_pose(self.wrist_p, self.wrist_q, pos_error)
        self.cmd_wrist_abs(wrist_init_pose, True)

        # spawn new object and close
        self.spawn_object()
        self.close_until_contact_and_tighten()
        self.wait_until_grasp_stabilizes()

        self.sim_pause()

    def close_until_contact_and_tighten(self, tighten_incr=0.1):
        res = self.close_until_contact(TriggerRequest())
        if self.verbose:
            rospy.loginfo("Closed reflex fingers until contact: \n" + str(res))
        res = self.pos_incr(tighten_incr, tighten_incr, tighten_incr, 0, False, False, 0, 0)
        if self.verbose:
            rospy.loginfo("Tightened fingers by " + str(tighten_incr) + ": \n" + str(res))

    def regrasp(self, wrist_p_incr, wrist_q_incr, back_off_finger=-0.2):
        self.sim_unpause()

        # back off fingers by a small amount
        res = self.pos_incr(back_off_finger, back_off_finger, back_off_finger, 0, True, True, self.srv_tolerance, self.srv_time_out)

        # apply relative wrist increment
        self.cmd_wrist_pose_incr(wrist_p_incr, wrist_q_incr)
        self.close_until_contact_and_tighten()
        self.wait_until_grasp_stabilizes()

        self.sim_pause()

    def object_lifted(self):
        msg = rospy.wait_for_message("/gazebo/object_sensor_bumper", ContactsState)
        collision_name = "ground_plane::link::collision"

        for i in range(len(msg.states)):
            if msg.states[i].collision1_name == collision_name or msg.states[i].collision2_name == collision_name:
                return False
        return True

    def get_wrist_init_pose(self, wrist_p, wrist_q, pos_error):
        self.wrist_init_pose = get_homo_matrix_from_tq(wrist_p, wrist_q)
        # generate random offset from initial wrist pose
        x_offset = np.random.uniform(min(pos_error[0]), max(pos_error[0]))
        y_offset = np.random.uniform(min(pos_error[1]), max(pos_error[1]))
        z_offset = np.random.uniform(min(pos_error[2]), max(pos_error[2]))
        rospy.loginfo(f"Random offset for init wrist pose is [x: {x_offset}, y: {y_offset}, z: {z_offset}].")
        mat_offset = tf.transformations.translation_matrix([x_offset, y_offset, z_offset])
        return np.dot(self.wrist_init_pose, mat_offset)

    def select_random_object_wrist_pair(self):
        # select new object randomly
        self.desired_obj_name = random.choice(self.object_names)

        # get object and grasp pose from yaml file
        self.obj_p = rospy.get_param(f"{self.desired_obj_name}/object_p")
        self.obj_q = rospy.get_param(f"{self.desired_obj_name}/object_q")
        self.wrist_p = rospy.get_param(f"{self.desired_obj_name}/wrist_p")
        self.wrist_q = rospy.get_param(f"{self.desired_obj_name}/wrist_q")

    def spawn_object(self):
        # use service to spawn urdf model
        req = SpawnModelRequest()
        req.model_name = "object"
        req.model_xml = open(
            f"/home/parallels/catkin_ws/src/grasp_refinement/reflex_stack/simulation/description/urdf/objects/{self.desired_obj_name}.urdf", "r"
        ).read()
        req.reference_frame = "world"
        req.initial_pose = Pose(
            Point(self.obj_p[0], self.obj_p[1], self.obj_p[1]), Quaternion(self.obj_q[0], self.obj_q[1], self.obj_q[2], self.obj_q[3])
        )
        self.service_call_with_retries(self.spawn_model, req, self.spawn_model_name)
