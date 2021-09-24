from enum import Enum
import threading

import tf
import rospy
import numpy as np

from .helpers.transforms import get_tq_from_homo_matrix


class Stage(Enum):
    REFINE = 0
    LIFT = 1
    HOLD = 2
    END = 3


class StageController:
    def __init__(self, hparams, state, gazebo_interface):
        self.hparams = hparams
        self.state = state
        self.gi = gazebo_interface

    def safe_thread_start(self, thread, tries_left=100, wait=0.1):
        # sometimes we have problems starting a new thread, so wrapping in try, catch
        if not tries_left:
            return 
        try:
            thread.start()
        except Exception as e:
            rospy.logwarn(f"Failed to start thread: {e}. Trying again. Tries left: {tries_left}")
            rospy.sleep(wait)
            self.safe_thread_start(thread, tries_left=tries_left - 1)

    def update_stage(self):
        # check if we're done early
        if self.state.stage == Stage.REFINE and self.end_refinement_early():
            self.state.stage = Stage.END
            rospy.loginfo("Ending episode early while refining.")
        elif self.state.stage == Stage.HOLD and not self.is_object_lifted():
            self.state.stage = Stage.END
            rospy.loginfo("Object dropped while holding! New stage is END. :-(")

        # check which stage we're in based on cur_time_step
        elif self.state.cur_time_step == self.hparams["refine_steps"] and self.state.stage == Stage.REFINE:
            rospy.loginfo("Done with %i refine steps. New stage is LIFT.", self.hparams["refine_steps"])
            self.state.cur_time_step = 0
            self.state.stage = Stage.LIFT
            self.lift_thread = threading.Thread(target=self.lift_object)
            self.safe_thread_start(self.lift_thread)
        elif self.state.cur_time_step == self.hparams["lift_steps"] and self.state.stage == Stage.LIFT:
            self.lift_thread.join()  # wait for lifting to be done
            self.state.sustained_lifting = self.is_object_lifted()
            rospy.loginfo("Done with %i lift steps.", self.hparams["lift_steps"])
            self.state.cur_time_step = 0
            if not self.state.sustained_lifting:
                rospy.loginfo("Object dropped while lifting! New stage is END. :-(")
                self.state.stage = Stage.END
            else:
                rospy.loginfo("Object lifted! New stage is HOLD. :-)")
                self.state.stage = Stage.HOLD
                self.hold_thread = threading.Thread(target=self.hold_object)
                self.safe_thread_start(self.hold_thread)
        elif self.state.cur_time_step == self.hparams["hold_steps"] and self.state.stage == Stage.HOLD:
            self.hold_thread.join()  # wait for holding to be done (to get holding outcome)
            self.state.sustained_holding = self.is_object_lifted()
            rospy.loginfo("Done with %i hold steps. New stage is END.", self.hparams["hold_steps"])
            self.state.stage = Stage.END

        self.state.cur_time_step += 1

    def end_refinement_early(self):
        # get object shift and distance to object
        obj_t, _ = get_tq_from_homo_matrix(self.gi.get_object_pose())
        self.state.obj_shift = np.linalg.norm(obj_t - self.gi.start_obj_t)
        self.state.dist_tcp_obj = self.gi.get_dist_tcp_obj()
        if self.state.obj_shift > self.hparams["obj_shift_tol"]:
            rospy.loginfo(f"Object shift is above {self.hparams['obj_shift_tol']} m.")
            return True
        elif not all(prox_angle < self.hparams["joint_lim"] for prox_angle in self.state.prox_angles):
            rospy.loginfo(f"One angle is above {self.hparams['joint_lim']} rad.")
            return True
        return False

    def lift_object(self):
        counter = 0
        rate = 1000  # pretty high update rate because we want incremental steps to be small for smooth lift off
        r = rospy.Rate(rate)
        z_incr = self.hparams["lift_dist"] / (self.hparams["secs_to_lift"] * rate)

        rospy.loginfo("Starting to lift object.")
        while counter * z_incr <= self.hparams["lift_dist"]:
            lift_mat = tf.transformations.translation_matrix([0, 0, z_incr])
            lift_mat = np.dot(lift_mat, self.gi.last_wrist_pose)
            self.gi.cmd_wrist_abs(lift_mat)
            r.sleep()
            counter += 1

    def hold_object(self):
        rospy.loginfo("Starting to hold object.")
        rospy.sleep(self.hparams["secs_to_hold"])

    def is_object_lifted(self):
        with self.state.mutex:
            return self.state.object_lifted
