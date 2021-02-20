import numpy as np
import tf


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
