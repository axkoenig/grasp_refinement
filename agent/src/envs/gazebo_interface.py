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

        self.br = tf2_ros.TransformBroadcaster()
        self.ts_wrist = geometry_msgs.msg.TransformStamped()

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

    def get_homo_matrix_from_msg(self, pose):
        trans = tf.transformations.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        rot = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return np.dot(trans, rot)

    def get_model_pose(self, model, frame="world"):
        try:
            res = self.get_model_state(model, frame)
        except rospy.ServiceException as e:
            rospy.loginfo(self.get_model_state_name + " service call failed with exception: " + str(e))
        return self.get_homo_matrix_from_msg(res.pose)

    def get_link_pose(self, link, frame="world"):
        try:
            res = self.get_link_state(link, frame)
        except rospy.ServiceException as e:
            rospy.loginfo(self.get_link_state_name + " service call failed with exception: " + e)
        return self.get_homo_matrix_from_msg(res.link_state.pose)

    def get_dist_tcp_obj(self):
        mat_shell = self.get_link_pose("shell")
        mat_obj = self.get_model_pose(self.object_name)
   
        # add tcp offset to current shell transform
        mat_tcp = tf.transformations.translation_matrix([0.02, 0, 0.09228])
        mat_tcp = np.dot(mat_shell, mat_tcp)
   
        return np.linalg.norm(tf.transformations.translation_from_matrix(mat_tcp) - tf.transformations.translation_from_matrix(mat_obj))

    def move_wrist_along_z(self, increment):
        mat_shell = self.get_link_pose("shell")
        mat_incr = tf.transformations.translation_matrix([0,0,increment])
        
        # add increment to cur wrist pose
        mat_shell = np.dot(mat_shell, mat_incr)
        t = tf.transformations.translation_from_matrix(mat_shell)
        q = tf.transformations.quaternion_from_matrix(mat_shell)
        
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

    def reset_object(self):

        # move to start pos

        # wait until reached start pos

        # reset sphere

        # close fingers until contact
        pass
