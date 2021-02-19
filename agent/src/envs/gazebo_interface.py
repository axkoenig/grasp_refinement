import numpy as np
import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, GetLinkState


class GazeboInterface:
    def __init__(self, verbose=False):

        self.verbose = verbose
        self.unpause_name = "/gazebo/unpause_physics"
        self.pause_name = "/gazebo/pause_physics"
        self.get_model_state_name = "/gazebo/get_model_state"
        self.get_link_state_name = "/gazebo/get_link_state"
        self.object_name = rospy.get_param("/object_name")

        self.unpause = rospy.ServiceProxy(self.unpause_name, Empty)
        self.pause = rospy.ServiceProxy(self.pause_name, Empty)
        self.get_model_state = rospy.ServiceProxy(self.get_model_state_name, GetModelState)
        self.get_link_state = rospy.ServiceProxy(self.get_link_state_name, GetLinkState)

        br = tf2_ros.TransformBroadcaster()
        transform_wrist = geometry_msgs.msg.TransformStamped()

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

    def tq_from_pose(self, pose, name, frame):
        t = [pose.position.x, pose.position.y, pose.position.z]
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if self.verbose:
            print("Pose of {} relative to {} frame is \n t={}, q={}".format(name, frame, t, q))
        return np.array(t), np.array(q)

    def get_model_pose(self, model, frame="world"):
        try:
            res = self.get_model_state(model, frame)
        except rospy.ServiceException as e:
            rospy.loginfo(self.get_model_state_name + " service call failed with exception: " + str(e))
        return self.tq_from_pose(res.pose, model, frame)

    def get_link_pose(self, link, frame="world"):
        try:
            res = self.get_link_state(link, frame)
        except rospy.ServiceException as e:
            rospy.loginfo(self.get_link_state_name + " service call failed with exception: " + e)
        return self.tq_from_pose(res.link_state.pose, link, frame)

    def get_dist_tcp_obj(self):
        t_shell, q_shell = self.get_link_pose("shell")
        t_obj, _ = self.get_model_pose(self.object_name)

        # rotate tcp offset to current shell (i.e. wrist) orientation
        t_mat_tcp = tf.transformations.translation_matrix([0.02, 0, 0.09228])
        q_mat_shell = tf.transformations.quaternion_matrix(q_shell)
        t_mat_tcp = np.dot(q_mat_shell, t_mat_tcp)
        t_tcp = t_shell + tf.transformations.translation_from_matrix(t_mat_tcp)

        return np.linalg.norm(t_obj - t_tcp)

    def move_wrist_along_z(self, increment):
        # get wrist pose
        t, q = self.get_link_pose("reflex")

        # reflex z increment in homogeneous coordinates
        reflex_z = [0, 0, increment, 0]
        rot_mat = tf.transformations.quaternion_matrix(q)
        rotated_z = np.dot(rot_mat, reflex_z)

        # rotate z vector according to quaternion

        # add increment to cur wrist pose

        # transform broadcaster
        # transform_wrist.header.stamp = rospy.Time.now()
        # transform_wrist.header.frame_id = "world"
        # transform_wrist.child_frame_id = "reflex"
        # transform_wrist.transform.translation.x = 0
        # transform_wrist.transform.rotation.x = q[0]
        # br.sendTransform(transform_wrist)
        pass

    def reset_object(self):

        # move to start pos

        # wait until reached start pos

        # reset sphere

        # close fingers until contact
        pass
