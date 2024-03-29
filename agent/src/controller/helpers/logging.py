import numpy as np
import rospy
from ..stage import Stage


def merge_dicts(dict_1, dict_2):
    "Merges two dicts. If key exists value be saved in a numpy array, else we create a new key."
    if not dict_1 and not dict_2:
        return {}
    if not dict_2:
        return dict_1
    elif not dict_1:
        return dict_2

    for key, value in dict_2.items():
        try:  # add value as array
            dict_1[key] = np.append(dict_1[key], value)
        except KeyError:  # we have a new entry
            dict_1[key] = value
    return dict_1


def log_dict(dict, logger, prefix="", type=None, exclude_keys_from_logging=[]):
    for key, value in dict.items():
        if not key in exclude_keys_from_logging:
            try:
                if type == "mean":
                    logger.record(prefix + key, np.mean(value))
                elif type == "sum":
                    logger.record(prefix + key, np.sum(value))
                elif not type:
                    logger.record(prefix + key, value)
            except TypeError:
                print(f"Can't compute {type} of {key} of type {type(value)}. Skipping this entry!")


def get_done_or_dones(callback_obj):
    # depending on which algorithm we're using we want to access "done" (e.g. td3) or "dones" (e.g. ppo)
    try_list = ["done", "dones"]
    for try_item in try_list:
        try:
            return callback_obj.locals[try_item][0]
        except KeyError:
            pass


def get_joint_difference(prox_angles):
    return abs(prox_angles[0] - prox_angles[1]) + abs(prox_angles[1] - prox_angles[2]) + abs(prox_angles[0] - prox_angles[2])


def get_quality_metrics_dict(state, prefix="", suffix=""):
    if suffix:
        suffix = "_" + suffix
    return {
        f"{prefix}_epsilon_force{suffix}": state.epsilon_force,
        f"{prefix}_epsilon_torque{suffix}": state.epsilon_torque,
        f"{prefix}_delta_task{suffix}": state.delta_task,
        f"{prefix}_delta_cur{suffix}": state.delta_cur,
    }


def get_infos(state, verbose=True):
    with state.mutex:
        # log the below data in each time step
        infos = {
            "num_contacts": state.num_contacts,
            "sum_contact_forces": state.sum_contact_forces,
            "joint_diff": get_joint_difference(state.prox_angles),
        }

        infos.update(get_quality_metrics_dict(state, "a"))
        # log again but this time specify object type
        infos.update(get_quality_metrics_dict(state, "a", state.cur_test_case.object.type))

        # log data that is stage-specific
        if state.stage == Stage.REFINE:
            infos.update(
                {
                    "r_dist_tcp_obj": state.dist_tcp_obj,
                    "r_obj_shift": state.obj_shift,
                }
            )
            infos.update(get_quality_metrics_dict(state, "r"))
        elif state.stage == Stage.LIFT:
            infos.update(get_quality_metrics_dict(state, "l"))
        elif state.stage == Stage.HOLD:
            infos.update(get_quality_metrics_dict(state, "h"))
        elif state.stage == Stage.END:
            infos.update(
                {
                    "sustained_holding": state.sustained_holding,
                    "sustained_lifting": state.sustained_lifting,
                    f"sustained_holding_{state.cur_test_case.object.type}": state.sustained_holding,
                    f"sustained_lifting_{state.cur_test_case.object.type}": state.sustained_lifting,
                }
            )

    if verbose:
        rospy.loginfo("--- Infos ---")
        for key, value in infos.items():
            rospy.loginfo(f"- {key:20}{value}")
    return infos
