import numpy as np
import rospy
import tf


def get_homo_matrix_from_transform_msg(transform, name, frame, verbose=False):
    t = [transform.translation.x, transform.translation.y, transform.translation.z]
    q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    if verbose:
        print("Pose of {} relative to {} frame is \n t={}, q={}".format(name, frame, t, q))
    mat_t = tf.transformations.translation_matrix(t)
    mat_q = tf.transformations.quaternion_matrix(q)
    return np.dot(mat_t, mat_q)


def get_homo_matrix_from_pose_msg(pose):
    t = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return get_homo_matrix_from_tq(t, q)


def get_homo_matrix_from_tq(t, q):
    mat_t = tf.transformations.translation_matrix(t)
    mat_q = tf.transformations.quaternion_matrix(q)
    return np.dot(mat_t, mat_q)


def get_tq_from_homo_matrix(mat):
    t = tf.transformations.translation_from_matrix(mat)
    q = tf.transformations.quaternion_from_matrix(mat)
    return t, q


def rad2deg(rad):
    return rad / np.pi * 180


def deg2rad(deg):
    return deg * np.pi / 180


def service_call(service, request, service_name):
    rospy.wait_for_service(service_name)
    try:
        if request.__class__.__name__ is "StringServiceRequest":
            res = service(request.part_name, request.reference_frame)
            return True, res
        else:
            res = service(request) if request else service()
            return True, res
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call to {service_name} failed with exception: {str(e)}")
        return False, None


def service_call_with_retries(service, request, max_retries=10):
    tries = 0
    service_name = service.protocol.resolved_name
    while tries < max_retries:
        success, res = service_call(service, request, service_name)
        if success:
            return res
        rospy.loginfo(f"Service call to {service_name} failed. Trying again ...")
        tries += 1
    rospy.loginfo(f"Service call to {service_name} failed even after {max_retries}.")


# some Gazebo services take strings as requests instead of request objects (hence creating an request object to make compatible with other code)
class StringServiceRequest:
    def __init__(self, part_name, reference_frame):
        self.part_name = part_name
        self.reference_frame = reference_frame