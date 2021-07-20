import numpy as np
import tf


def normalize(val, low_bound, high_bound):
    return (val - low_bound) / (high_bound - low_bound)


def normalize_list_items(list_val, list_low_bound, list_high_bound):
    is_float = type(list_val) is float or type(list_low_bound) is float or type(list_high_bound) is float
    is_int = type(list_val) is int or type(list_low_bound) is int or type(list_high_bound) is int
    if is_int or is_float:
        return normalize(list_val, list_low_bound, list_high_bound)
    for i in range(len(list_val)):
        list_val[i] = normalize(list_val[i], list_low_bound[i], list_high_bound[i])
    return list_val


def is_valid_vertical_offset(wrist_y_offset, object_height, ground_clearance=0.02):
    # prevents random wrist poses that would touch ground (wrist_y_offset in shell frame and object z in world coosy!)
    # TODO: this is a bit hacky, better work with transforms in world coosy, but we don't always know them a priori
    # warning: this function assumes that reflex y axis pointing into negative world z
    # warning2: we don't take the rotation into account here (since it is usually small)
    shell_box_height = 0.084  # this comes from urdf
    wrist_z_in_world = -wrist_y_offset  # since we defined that above
    return (object_height / 2 + wrist_z_in_world - shell_box_height / 2) >= ground_clearance


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
