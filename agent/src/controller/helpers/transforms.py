import numpy as np
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
