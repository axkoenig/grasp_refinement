from multiprocessing import Lock
import threading

import gym
import numpy as np
import rospy
import tf

from .helpers.transforms import get_tq_from_homo_matrix
from .helpers.logging import get_infos
from .spaces.space_act import ActionSpace
from .spaces.space_obs import ObservationSpace
from .gazebo_interface import GazeboInterface
from .subscribers import Subscribers
from .stage import Stage
from .state import State
from .rewards import Rewards


class GazeboEnv(gym.Env):
    def __init__(self, hparams, name="TRAIN"):

        self.hparams = hparams
        self.name = name
        self.mutex = Lock()

        # counters
        self.cur_time_step = 0

        self.state = State()
        self.gi = GazeboInterface(verbose=False)
        self.acts = ActionSpace()
        self.obs = ObservationSpace()
        self.sub = Subscribers(self.state, self.obs, self.gi)
        self.rewards = Rewards(self.hparams, self.state)

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals())
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals())
        self.reward_range = (-np.inf, np.inf)

        # update rates
        self.rate_lift = self.hparams["lift_steps"] / self.hparams["secs_to_lift"]
        self.rate_hold = self.hparams["hold_steps"] / self.hparams["secs_to_hold"]
        rospy.loginfo("Rate lift is: \t%f", self.rate_lift)
        rospy.loginfo("Rate hold is: \t%f", self.rate_hold)

    ### GYM METHODS

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-{self.state.stage.name}-STEP:[{self.cur_time_step}]===")

        action_dict = self.acts.get_action_dict(action, verbose=True)
        self.act(action_dict)

        reward = self.rewards.get_reward_rlh(self.get_exec_secs())

        self.update_stage()
        self.done = True if self.state.stage == Stage.END else False

        if self.done:
            reward += self.get_reward_end()

        rospy.loginfo(f"--> REWARD: \t {reward}")

        return self.obs.get_cur_vals(), reward, self.done, get_infos(self.state)

    def reset(self):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-RESETTING===")
        self.gi.reset_world(self.hparams)
        self.last_time_stamp = rospy.Time.now()
        self.cur_time_step = 0
        self.state.reset()
        self.gi.sim_pause()  # when NN is updating after resetting, we pause simulation
        return self.obs.get_cur_vals()

    ### OTHER METHODS

    def get_exec_secs(self):
        # this is to wait 80% of one time step to collect reward
        # we leave 20% for other code to run (later on we make sure we get the exact update rate via the wait_if_necessary method)
        return 0.8 * (1 / self.get_rate_of_cur_stage())

    def act(self, action_dict):
        if action_dict["trigger_regrasp"]:
            # we only allow wrist control during refinement
            if self.state.stage == Stage.REFINE:
                rospy.loginfo(">>REGRASPING<<")
                rot = action_dict["wrist_rot"]
                wrist_q = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
                self.gi.regrasp(action_dict["wrist_trans"], wrist_q, self.state.prox_angles)
                self.state.num_regrasps += 1
            else:
                rospy.loginfo(">>STAYING<<")
        else:
            rospy.loginfo(">>ADJUSTING FINGERS<<")
            self.wait_if_necessary()
            f = action_dict["fingers_incr"]
            self.gi.pos_incr(f[0], f[1], f[2], 0, False, False, 0, 0)

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
        d = rospy.Time.now() - self.last_time_stamp
        while rospy.Time.now() - self.last_time_stamp < rospy.Duration(step_size):
            rospy.loginfo_throttle(
                step_size, "Your last %s step only took %f seconds. Waiting to keep min step size of %f", self.state.stage.name, d.to_sec(), step_size
            )
            rospy.sleep(0.01)
        self.last_time_stamp = rospy.Time.now()


    def update_stage(self):
        # check if we're done early
        if self.state.stage == Stage.REFINE and self.end_refinement_early():
            self.state.stage = Stage.END
            rospy.loginfo("Ending episode early while refining.")
        elif self.state.stage == Stage.HOLD and not self.gi.object_lifted():
            self.state.stage = Stage.END
            rospy.loginfo("Object dropped while holding! New stage is END. :-(")

        # check which stage we're in based on cur_time_step
        elif self.cur_time_step == self.hparams["refine_steps"] and self.state.stage == Stage.REFINE:
            rospy.loginfo("Done with %i refine steps. New stage is LIFT.", self.hparams["refine_steps"])
            self.cur_time_step = 0
            self.state.stage = Stage.LIFT
            self.lift_thread = threading.Thread(target=self.lift_object)
            self.lift_thread.start()
        elif self.cur_time_step == self.hparams["lift_steps"] and self.state.stage == Stage.LIFT:
            self.lift_thread.join()  # wait for lifting to be done
            self.state.sustained_lifting = self.gi.object_lifted()
            rospy.loginfo("Done with %i lift steps.", self.hparams["lift_steps"])
            self.cur_time_step = 0
            if not self.state.sustained_lifting:
                rospy.loginfo("Object dropped while lifting! New stage is END. :-(")
                self.state.stage = Stage.END
            else:
                rospy.loginfo("Object lifted! New stage is HOLD. :-)")
                self.state.stage = Stage.HOLD
                self.hold_thread = threading.Thread(target=self.hold_object)
                self.hold_thread.start()
        elif self.cur_time_step == self.hparams["hold_steps"] and self.state.stage == Stage.HOLD:
            self.hold_thread.join()  # wait for holding to be done (to get holding outcome)
            self.state.sustained_holding = self.gi.object_lifted()
            rospy.loginfo("Done with %i hold steps. New stage is END.", self.hparams["hold_steps"])
            self.state.stage = Stage.END

        self.cur_time_step += 1

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
