import rospy
import tf
import numpy as np

from .stage import Stage


class Actions:
    def __init__(self, hparams, state, gazebo_interface):
        self.hparams = hparams
        self.state = state
        self.gi = gazebo_interface

        # update rates
        self.rate_lift = self.hparams["lift_steps"] / self.hparams["secs_to_lift"]
        self.rate_hold = self.hparams["hold_steps"] / self.hparams["secs_to_hold"]
        rospy.loginfo("Rate lift is: \t%f", self.rate_lift)
        rospy.loginfo("Rate hold is: \t%f", self.rate_hold)

    def act(self, action_dict):
        rot = action_dict["wrist_rot"]
        wrist_q = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
        self.gi.cmd_wrist_pose_incr(action_dict["wrist_trans"], wrist_q)
        f = action_dict["fingers_incr"]
        f = np.append(f, 0) # don't move preshape joint
        self.gi.finger_pos_incr(f)
        self.wait_if_necessary()

    def get_rate_of_cur_stage(self):
        if self.state.stage == Stage.REFINE:
            return self.hparams["max_refine_rate"]
        elif self.state.stage == Stage.LIFT:
            return self.rate_lift
        elif self.state.stage == Stage.HOLD:
            return self.rate_hold
        else:
            # there is no rate at stage END because it's only one time step
            return 1

    def wait_if_necessary(self):
        # makes sure that we are keeping desired update rate
        step_size = 1 / self.get_rate_of_cur_stage()
        d = rospy.Time.now() - self.state.last_time_stamp
        while rospy.Time.now() - self.state.last_time_stamp < rospy.Duration(step_size):
            rospy.loginfo_throttle(step_size, "Your last %s step only took %f seconds. Waiting to keep min step size of %f", self.state.stage.name, d.to_sec(), step_size)
            rospy.sleep(0.01)
        self.state.last_time_stamp = rospy.Time.now()
