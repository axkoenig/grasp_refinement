from enum import Enum
import threading

import gym
import numpy as np
import rospy
import tf

from reflex_interface.msg import HandStateStamped

from .helpers import get_tq_from_homo_matrix
from .space_act import ActionSpace
from .space_obs import ObservationSpace
from .gazebo_interface import GazeboInterface


class Stage(Enum):
    REFINE = 0
    LIFT = 1
    HOLD = 2
    END = 3


class GazeboEnv(gym.Env):
    def __init__(self, hparams, name="TRAIN"):

        self.hparams = hparams
        self.name = name

        # quality metrics
        self.epsilon_force = 0
        self.epsilon_torque = 0
        self.delta_task = 0
        self.delta_cur = 0
        self.sustained_holding = False
        self.sustained_lifting = False

        # counters
        self.num_regrasps = 0
        self.cur_time_step = 0
        self.stage = Stage(0)

        self.gi = GazeboInterface(verbose=False)
        self.acts = ActionSpace()
        self.obs = ObservationSpace()

        self.action_space = gym.spaces.Box(low=self.acts.get_min_vals(), high=self.acts.get_max_vals())
        self.observation_space = gym.spaces.Box(low=self.obs.get_min_vals(), high=self.obs.get_max_vals())
        self.reward_range = (-np.inf, np.inf)

        self.hand_state_sub = rospy.Subscriber("reflex_interface/hand_state", HandStateStamped, self.hand_state_callback, queue_size=5)

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
        self.update_stage()
        rospy.loginfo(f"==={self.name}-{self.stage.name}-STEP:[{self.cur_time_step}]===")

        action_dict = self.acts.get_action_dict(action, verbose=True)
        self.act(action_dict)

        reward = self.get_reward_quality_metrics()
        info = {}
        self.done = True if self.stage == Stage.END else False
        if self.done:
            reward += self.get_reward_binary()
            rospy.loginfo("Sustained holding: \t" + str(self.sustained_holding))
            rospy.loginfo("Sustained lifting: \t" + str(self.sustained_lifting))
            info = {"sustained_holding": self.sustained_holding, "sustained_lifting": self.sustained_lifting}

        rospy.loginfo(f"--> REWARD: \t {reward}")

        return self.obs.get_cur_vals(), reward, self.done, info

    def reset(self):
        self.gi.sim_unpause()
        rospy.loginfo(f"==={self.name}-RESETTING===")
        self.gi.reset_world(self.hparams)
        self.stage = Stage.REFINE
        self.start_reward = self.collect_reward(self.get_exec_secs())
        rospy.loginfo(f"Start reward is: \t{self.start_reward}")
        self.last_time_stamp = rospy.Time.now()
        self.cur_time_step = 0
        self.gi.sim_pause()  # when NN is updating after resetting, we pause simulation
        return self.obs.get_cur_vals()

    ### OTHER METHODS

    def act(self, action_dict):
        if self.stage == Stage.REFINE:
            if action_dict["trigger_regrasp"]:
                rospy.loginfo(">>REGRASPING<<")
                rot = action_dict["wrist_rot"]
                wrist_q = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
                self.gi.regrasp(action_dict["wrist_trans"], wrist_q, self.prox_angles)
                self.num_regrasps += 1  # we reset this var in the tensorboard callback once recorded
            else:
                rospy.loginfo(">>WRIST STAYING<<")
                self.adjust_fingers(action_dict)
        else:
            self.adjust_fingers(action_dict)

    def get_rate_of_cur_stage(self):
        if self.stage == Stage.REFINE:
            return self.hparams["max_refine_rate"]
        elif self.stage == Stage.LIFT:
            return self.rate_lift
        elif self.stage == Stage.HOLD:
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
                step_size, "Your last %s step only took %f seconds. Waiting to keep min step size of %f", self.stage.name, d.to_sec(), step_size
            )
            rospy.sleep(0.01)
        self.last_time_stamp = rospy.Time.now()

    def hand_state_callback(self, msg):
        self.num_contacts = msg.num_contacts
        self.epsilon_force = msg.epsilon_force
        self.epsilon_torque = msg.epsilon_torque
        self.delta_task = msg.delta_task
        self.delta_cur = msg.delta_cur
        self.sum_contact_forces = msg.sum_contact_forces
        self.prox_angles = [0, 0, 0]

        self.obs.set_cur_val_by_name("preshape_angle", msg.preshape_angle)

        for i in range(self.obs.num_fingers):
            # joint positions
            id_str = "_f" + str(i + 1)
            self.prox_angles[i] = msg.finger_state[i].proximal_angle
            self.obs.set_cur_val_by_name("prox_angle" + id_str, msg.finger_state[i].proximal_angle)
            self.obs.set_cur_val_by_name("dist_angle" + id_str, msg.finger_state[i].distal_angle)

            # normals
            normal = self.gi.ros_vector_to_list(msg.finger_state[i].prox_normal)
            self.obs.set_cur_val_by_name("prox_normal" + id_str, normal)
            normal = self.gi.ros_vector_to_list(msg.finger_state[i].dist_normal)
            self.obs.set_cur_val_by_name("dist_normal" + id_str, normal)

            # tactile feedback
            for j in range(self.obs.num_sensors):
                id_str = "_f" + str(i + 1) + "_s" + str(j + 1)
                self.obs.set_cur_val_by_name("sensor_pressure" + id_str, msg.finger_state[i].sensor_pressure[j])

                if msg.finger_state[i].sensor_contact[j]:
                    tactile_position = self.gi.ros_vector_to_list(msg.finger_state[i].tactile_position[j])
                    self.obs.set_cur_val_by_name("tactile_position" + id_str, tactile_position)
                    self.obs.set_cur_val_by_name("tactile_contact" + id_str, 1)

                else:
                    self.obs.set_cur_val_by_name("tactile_position" + id_str, self.obs.tactile_pos_default)
                    self.obs.set_cur_val_by_name("tactile_contact" + id_str, 0)

    def collect_reward(self, duration, rate=60):
        # records average reward over duration
        reward = 0
        counter = 0
        r = rospy.Rate(rate)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration.from_sec(duration):
            counter += 1
            reward += self.calc_reward()
            r.sleep()
        return reward / counter

    def calc_reward(self, w_eps_torque=10, w_delta=0.1):
        reward = self.epsilon_force + w_eps_torque * self.epsilon_torque
        reward += w_delta * self.delta_task if self.stage == Stage.REFINE else w_delta * self.delta_cur
        return reward

    def adjust_fingers(self, action_dict):
        self.wait_if_necessary()
        if action_dict["trigger_fingers"]:
            rospy.loginfo(">>ADJUSTING FINGERS<<")
            f = action_dict["fingers_incr"]
            self.gi.pos_incr(f[0], f[1], f[2], 0, False, False, 0, 0)

    def get_exec_secs(self):
        # this is to wait 80% of one time step to collect reward
        # we leave 20% for other code to run (later on we make sure we get the exact update rate via the wait_if_necessary method)
        return 0.8 * (1 / self.get_rate_of_cur_stage())

    def get_reward_quality_metrics(self):
        exec_secs = self.get_exec_secs()
        if self.hparams["framework"] == 1 or self.hparams["framework"] == 3:
            return self.collect_reward(exec_secs) - self.start_reward
        else:
            rospy.sleep(exec_secs)
            return 0

    def get_reward_binary(self, w_binary_rew=2):
        if self.hparams["framework"] == 2:
            return int(self.sustained_holding)
        elif self.hparams["framework"] == 3:
            return w_binary_rew * int(self.sustained_holding)
        else:
            return 0

    def update_stage(self):
        # check if we're done early
        if self.stage == Stage.REFINE and self.end_refinement_early():
            self.stage = Stage.END
            rospy.loginfo("Ending episode early while refining.")
        elif self.stage == Stage.HOLD and not self.gi.object_lifted():
            self.stage = Stage.END
            self.sustained_holding = False
            rospy.loginfo("Ending episode early while holding. Dropped object! :-(")

        # check which stage we're in based on cur_time_step
        elif self.cur_time_step == self.hparams["refine_steps"] and self.stage == Stage.REFINE:
            rospy.loginfo("Done with %i refine steps. New stage is LIFT.", self.hparams["refine_steps"])
            self.cur_time_step = 0
            self.stage = Stage.LIFT
            self.lift_thread = threading.Thread(target=self.lift_object)
            self.lift_thread.start()
        elif self.cur_time_step == self.hparams["lift_steps"] and self.stage == Stage.LIFT:
            self.lift_thread.join()  # wait for lifting to be done
            rospy.loginfo("Done with %i lift steps.", self.hparams["lift_steps"])
            self.cur_time_step = 0
            if not self.sustained_lifting:
                self.sustained_holding = False  # we can't hold if we dropped it
                rospy.loginfo("Object dropped while lifting! New stage is END. :-(")
                self.stage = Stage.END
            else:
                rospy.loginfo("Object lifted! New stage is HOLD. :-)")
                self.stage = Stage.HOLD
                self.hold_thread = threading.Thread(target=self.hold_object)
                self.hold_thread.start()
        elif self.cur_time_step == self.hparams["hold_steps"] and self.stage == Stage.HOLD:
            rospy.loginfo("Done with %i hold steps. New stage is END.", self.hparams["hold_steps"])
            self.hold_thread.join()  # wait for holding to be done (to get holding outcome)
            self.stage = Stage.END

        self.cur_time_step += 1

    def end_refinement_early(self):
        # get object shift and distance to object
        obj_t, _ = get_tq_from_homo_matrix(self.gi.get_object_pose())
        self.obj_shift = np.linalg.norm(obj_t - self.gi.start_obj_t)
        self.dist_tcp_obj = self.gi.get_dist_tcp_obj()
        if self.obj_shift > self.hparams["obj_shift_tol"]:
            rospy.loginfo(f"Object shift is above {self.hparams['obj_shift_tol']} m.")
            return True
        elif not all(prox_angle < self.hparams["joint_lim"] for prox_angle in self.prox_angles):
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

        self.sustained_lifting = self.gi.object_lifted()

    def hold_object(self):
        rospy.loginfo("Starting to hold object.")
        rospy.sleep(self.hparams["secs_to_hold"])
        self.sustained_holding = self.gi.object_lifted()