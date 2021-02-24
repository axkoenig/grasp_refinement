import numpy as np
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from std_srvs.srv import Empty, Trigger, TriggerRequest
from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState
from gazebo_msgs.msg import ModelState

from .helpers import get_tq_from_homo_matrix


class GazeboInterface:
    def __init__(self, verbose=False):
        self.verbose = verbose

        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.get_model_state_name = "/gazebo/get_model_state"
        self.get_link_state_name = "/gazebo/get_link_state"
        self.set_model_state_name = "/gazebo/set_model_state"
        self.object_name = rospy.get_param("/object_name")

        # gazebo services
        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)
        self.get_model_state = rospy.ServiceProxy(self.get_model_state_name, GetModelState)
        self.get_link_state = rospy.ServiceProxy(self.get_link_state_name, GetLinkState)
        self.set_model_state = rospy.ServiceProxy(self.set_model_state_name, SetModelState)

        # reflex services
        self.open_hand = rospy.ServiceProxy("/reflex/open", Trigger)
        self.sph_open_hand = rospy.ServiceProxy("/reflex/spherical_open", Trigger)
        self.close_until_contact = rospy.ServiceProxy("/reflex/close_until_contact", Trigger)

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
        try:
            self.unpause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.unpause_name + " service call failed with exception: " + str(e))

    def sim_pause(self):
        try:
            self.pause()
        except rospy.ServiceException as e:
            rospy.loginfo(self.pause_name + " service call failed")

    def run_for_seconds(self, prefix, secs, cmd_str):
        self.sim_unpause()
        rospy.loginfo(f"{prefix}: Executing " + cmd_str + f" for {secs} secs.")
        rospy.sleep(secs)
        self.sim_pause()

    def get_homo_matrix_from_msg(self, transform, name, frame):
        t = [transform.translation.x, transform.translation.y, transform.translation.z]
        q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        if self.verbose:
            print("Pose of {} relative to {} frame is \n t={}, q={}".format(name, frame, t, q))
        mat_t = tf.transformations.translation_matrix(t)
        mat_q = tf.transformations.quaternion_matrix(q)
        return np.dot(mat_t, mat_q)

    def get_object_pose(self):
        return self.get_measured_pose(self.mes_obj_buf, self.mes_obj_name)

    def get_wrist_pose(self):
        return self.get_measured_pose(self.mes_wrist_buf, self.mes_wrist_name)

    def get_measured_pose(self, buffer, name, frame="world"):
        try:
            trans = buffer.lookup_transform(frame, name, rospy.Time())
            return self.get_homo_matrix_from_msg(trans.transform, name, frame)
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
        rospy.wait_for_service(self.set_model_state_name)
        try:
            res = self.set_model_state(state)
            rospy.loginfo(f"Successfully set state of {model_name}.")
        except rospy.ServiceException:
            rospy.loginfo(f"{self.set_model_state_name} service call failed.")

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

    def cmd_wrist_z_abs(self, origin_pose, origin_offset):
        mat_offset = tf.transformations.translation_matrix([0, 0, origin_offset])
        mat_shell = np.dot(origin_pose, mat_offset)
        self.send_transform(mat_shell)

    def cmd_wrist_pos_abs(self, origin_pose, origin_offset):
        mat_offset = tf.transformations.translation_matrix(origin_offset)
        mat_shell = np.dot(origin_pose, mat_offset)
        self.send_transform(mat_shell)

    def cmd_wrist_pos_incr(self, increment):
        mat_increment = tf.transformations.translation_matrix(increment)
        self.last_wrist_pose = np.dot(self.last_wrist_pose, mat_increment)
        self.send_transform(self.last_wrist_pose)

    def wait_until_reached_pose(self, pose, t_tol=0.02, q_tol=0.05):
        r = rospy.Rate(5)
        t, q = get_tq_from_homo_matrix(pose)
        while not rospy.is_shutdown():
            mat_shell = self.get_wrist_pose()
            t_cur, q_cur = get_tq_from_homo_matrix(mat_shell)
            if np.linalg.norm(t_cur - t) > t_tol:
                rospy.loginfo(f"Wrist position not within tolerance of {t_tol} yet.")
            elif np.linalg.norm(q_cur - q) > t_tol:
                rospy.loginfo(f"Wrist orientation not within tolerance of {q_tol} yet.")
            else:
                rospy.loginfo("Wrist reached target position.")
                return
            r.sleep()

    def reset_world(self, mat_shell, mat_obj):
        self.sim_unpause()

        res = self.open_hand(TriggerRequest())
        rospy.sleep(0.5)
        rospy.loginfo("Opened reflex fingers: \n" + str(res))

        rospy.loginfo("Moving wrist to start position.")
        self.last_wrist_pose = mat_shell
        self.send_transform(mat_shell)
        self.wait_until_reached_pose(mat_shell)
        self.set_model_pose(mat_obj, self.object_name)

        res = self.sph_open_hand(TriggerRequest())
        rospy.sleep(0.5)
        rospy.loginfo("Spherical openend reflex fingers: \n" + str(res))

        res = self.close_until_contact(TriggerRequest())
        rospy.loginfo("Closed reflex fingers until contact: \n" + str(res))
        rospy.sleep(1)
        self.sim_pause()
