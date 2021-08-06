import os
import time

import numpy as np
import rospy
import rosnode
import roslib
import tf2_ros
import roslaunch
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_srvs.srv import Empty, Trigger, TriggerRequest
from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState, DeleteModel, SpawnModel, DeleteModelRequest, SpawnModelRequest, GetWorldProperties
from gazebo_msgs.msg import ModelState, ContactsState

from reflex_interface.srv import PosIncrement, GraspPrimitive
from .helpers.transforms import get_tq_from_homo_matrix, get_homo_matrix_from_tq, get_homo_matrix_from_pose_msg, deg2rad
from .helpers.services import StringServiceRequest, service_call_with_retries
from .tests import TestCaseFromRanges, gen_valid_wrist_error_from_l2


class GazeboInterface:
    def __init__(self, hparams, verbose=True):
        self.verbose = verbose
        self.hparams = hparams
        self.description_path = roslib.packages.get_pkg_dir("description")

        # gazebo services
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

        # reflex services
        self.open_hand = rospy.ServiceProxy("/reflex_interface/open", GraspPrimitive)
        self.close_until_contact = rospy.ServiceProxy("/reflex_interface/close_until_contact", Trigger)
        self.pos_incr = rospy.ServiceProxy("/reflex_interface/position_increment", PosIncrement)

        # some vars
        self.num_srv_tries = 0
        self.srv_time_out = 5
        self.srv_tolerance = 0.01
        self.resetting_attempts = 0
        self.knocked_obj_over = False

        # setup broadcaster for desired wrist pose
        self.ts_wrist = TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.sim_unpause()  # make sure simulation is not paused
        rospy.sleep(1)  # broadcaster needs some time to start

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

    def finger_pos_incr(self, incr, from_measured_pos=False, blocking=False, tolerance=0, timeout=0):
        req = StringServiceRequest((incr[0], incr[1], incr[2], incr[3], from_measured_pos, blocking, tolerance, timeout))
        service_call_with_retries(self.pos_incr, req)

    def get_ros_param_with_retries(self, param_name, time_out=10):
        r = rospy.Rate(5)
        begin = rospy.get_rostime()
        d = rospy.Duration.from_sec(time_out)
        while rospy.get_rostime() - begin < d:
            try:
                return rospy.get_param(param_name)
            except KeyError:
                rospy.logwarn(f"ROS parameter {param_name} not yet available. Waiting ...")
            r.sleep()
        rospy.logerr(f"Did not find ROS parameter {param_name} within time out of {time_out} secs.")
        raise KeyError

    def sim_unpause(self):
        service_call_with_retries(self.unpause_physics)

    def sim_pause(self):
        service_call_with_retries(self.pause_physics)

    def get_object_pose(self):
        object_name = self.get_cur_obj_name()
        req = StringServiceRequest((object_name, "world"))
        res = service_call_with_retries(self.get_model_state, req)
        if not res:  # sometimes we don't get a result (we handle this during resetting)
            rospy.logwarn("Could not get object pose! Returning identity")
            return tf.transformations.identity_matrix()
        return get_homo_matrix_from_pose_msg(res.pose)

    def get_wrist_pose(self):
        req = StringServiceRequest(("shell", "world"))
        res = service_call_with_retries(self.get_link_state, req)
        return get_homo_matrix_from_pose_msg(res.link_state.pose)

    def get_trans_tcp_obj(self):
        # add tcp offset to current shell transform
        mat_tcp = tf.transformations.translation_matrix([0.02, 0, 0.09228])
        mat_tcp = np.dot(self.get_wrist_pose(), mat_tcp)

        # get transform from tcp to object
        mat_tcp_to_obj = np.dot(tf.transformations.inverse_matrix(mat_tcp), self.get_object_pose())
        return tf.transformations.translation_from_matrix(mat_tcp_to_obj)

    def get_dist_tcp_obj(self):
        return np.linalg.norm(self.get_trans_tcp_obj())

    def set_model_pose(self, pose, model_name, reference_frame="world"):
        t, q = get_tq_from_homo_matrix(pose)
        self.set_model_pose_tq(t, q, model_name, reference_frame)

    def set_model_pose_tq(self, t, q, model_name, reference_frame="world"):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = reference_frame
        state.pose.position.x = t[0]
        state.pose.position.y = t[1]
        state.pose.position.z = t[2]
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]
        service_call_with_retries(self.set_model_state, state)

    def send_transform(self, pose):
        t, q = get_tq_from_homo_matrix(pose)
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

    def tcp_to_wrist(self):
        return tf.transformations.translation_matrix([-0.02, 0, -0.09228])

    def cmd_wrist_abs(self, mat_shell, wait_until_reached_pose=False, control_tcp=False):
        if control_tcp:
            mat_shell = np.dot(mat_shell, self.tcp_to_wrist())
        self.last_wrist_pose = mat_shell
        self.send_transform(mat_shell)
        if wait_until_reached_pose:
            return self.wait_until_reached_pose(mat_shell)
        return 1

    def cmd_wrist_pose_incr(self, t_incr, q_incr, wait_until_reached_pose=False):
        mat_homo = get_homo_matrix_from_tq(t_incr, q_incr)
        return self.cmd_wrist_abs(np.dot(self.last_wrist_pose, mat_homo), wait_until_reached_pose)

    def wait_until_reached_pose(self, pose, t_tol=0.01, q_tol=0.02, time_out=10):
        r = rospy.Rate(5)
        t, q = get_tq_from_homo_matrix(pose)
        begin = rospy.get_rostime()
        d = rospy.Duration.from_sec(time_out)
        while rospy.get_rostime() - begin < d:
            mat_shell = self.get_wrist_pose()
            t_cur, q_cur = get_tq_from_homo_matrix(mat_shell)
            if np.linalg.norm(t_cur - t) > t_tol and self.verbose:
                rospy.loginfo(f"Wrist position not within tolerance of {t_tol} yet.")
            elif np.linalg.norm(q_cur - q) > q_tol and self.verbose:
                rospy.loginfo(f"Wrist orientation not within tolerance of {q_tol} yet.")
            elif np.linalg.norm(t_cur - t) < t_tol and np.linalg.norm(q_cur - q) < q_tol:
                return 1
            r.sleep()
        rospy.loginfo(f"Did not reach pose in time out of {time_out} secs. You're probably crashing into something ...")
        return 0

    def wait_until_grasp_stabilizes(self):
        rospy.sleep(0.5)

    def delete_model(self, name="object"):
        req = DeleteModelRequest()
        req.model_name = name
        service_call_with_retries(self.delete_model, req)

    def get_cur_obj_name(self):
        res = service_call_with_retries(self.get_world_properties)
        models = res.model_names
        irrelevant_objects = ["ground_plane", "reflex", "sphere_mount"]
        for obj in irrelevant_objects:
            if obj in models:
                models.remove(obj)
        if len(models) != 1:
            rospy.logwarn("You have more or less than one model in your world! Returning '' for object name.")
            return ""
        return models[0]

    def shutdown_launch_files(self):
        rospy.loginfo("Shutting down launch files")
        try:
            self.scene_launch.shutdown()
        except AttributeError:
            pass  # when we reset the first time these vars don't exist yet, no need to worry

    def delete_all_models(self):
        # deletes all models except ground plane
        res = service_call_with_retries(self.get_world_properties)
        for model in res.model_names:
            if model != "ground_plane":
                self.delete_model(model)
                rospy.sleep(0.5)

    def launch_scene(self, object):
        rospy.loginfo("Launching reflex launch file ...")
        launch_file = roslib.packages.get_pkg_dir("description") + "/launch/reflex.launch"
        simplify_collisions = "true" if self.hparams["simplify_collisions"] else "false"
        cli_args = [
            launch_file,
            "spawn_new_world:=false",
            "output:=log",
            f"object_mass:={object.mass}",
            f"inertia_scaling_factor:={object.inertia_scaling_factor}",
            "origin_x:=0",
            "origin_y:=0.2",
            f"origin_z:={object.get_height() / 2 + 0.01}",
            f"object_name:={object.name}",
            f"simplify_collisions:={simplify_collisions}",
        ]

        if object.__class__.__name__ == "RandomSphere":
            cli_args += ["object_type:=sphere", f"sphere_radius:={object.radius}"]
        elif object.__class__.__name__ == "RandomCylinder":
            cli_args += ["object_type:=cylinder", f"cylinder_radius:={object.radius}", f"cylinder_length:={object.length}"]
        elif object.__class__.__name__ == "RandomBox":
            cli_args += ["object_type:=box", f"box_x:={object.x}", f"box_y:={object.y}", f"box_z:={object.z}"]

        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.scene_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        self.scene_launch.start()

    def wait_until_reflex_spawned(self, time_out=20):
        begin = time.time()
        while int(time.time() - begin) < time_out:
            res = service_call_with_retries(self.get_world_properties)
            if "reflex" in res.model_names:
                rospy.loginfo("Found reflex in world. All good!")
                return
            else:
                time.sleep(1)
        raise Exception("Reflex not found in world, something is wrong.")

    def wait_until_nodes_spawned(self, time_out=20):
        begin = time.time()
        while int(time.time() - begin) < time_out:
            res = rosnode.get_node_names()
            print(res)
            if "/finger_controller_node" in res and "/reflex_interface_node" in res and "/sensor_listener_node" in res and "/wrist_controller_node" in res:
                rospy.loginfo("All nodes are up!")
                return
            else:
                time.sleep(1)
        raise Exception("Not all nodes are up, something is wrong.")

    def reset_world(self, state, test_case=None):
        try:
            if test_case and (self.resetting_attempts > 20 or self.knocked_obj_over):
                rospy.logerr(
                    f"resetting_attempts {self.resetting_attempts}, knocked_obj_over {self.knocked_obj_over}. Generating a new wrist pose on this object."
                )
                test_case.wrist_error = gen_valid_wrist_error_from_l2(test_case.object, test_case.trans_l2_error, test_case.rot_l2_error, self.hparams)
                # reset vars for this new wrist error
                self.knocked_obj_over = False
                self.resetting_attempts = 0

            self.delete_all_models()
            self.shutdown_launch_files()
            self.cmd_wrist_abs(tf.transformations.identity_matrix())

            # generate own test case when training (this is technically a "train" case)
            if not test_case:
                rospy.loginfo("Generating new test case")
                test_case = TestCaseFromRanges(self.hparams)
                rospy.loginfo("Object is " + test_case.object.type)

            state.cur_test_case = test_case
            self.launch_scene(test_case.object)
            self.wait_until_reflex_spawned()
            self.wait_until_nodes_spawned()

            obj_pose = self.get_object_pose()
            if (obj_pose == tf.transformations.identity_matrix()).all():
                rospy.logwarn("Could not get object pose. Resetting again.")
                self.resetting_attempts += 1
                return self.reset_world(state, test_case)

            # calc ground truth pose of reflex (which is offset from object frame)
            obj_t, obj_q = get_tq_from_homo_matrix(obj_pose)
            truth_wrist_t = obj_t - [0, 0.05, 0]  # 5cm offset
            truth_wrist_q = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, 0)

            # move to waypoint poses
            wrist_waypoint_pose = get_homo_matrix_from_tq([0, 0, 0.12], truth_wrist_q)
            res = self.cmd_wrist_abs(wrist_waypoint_pose, True, True)
            if not res:
                rospy.logwarn("Could not reach waypoint wrist pose. Resetting again.")
                self.resetting_attempts += 1
                return self.reset_world(state, test_case)
            self.open_hand(True, self.srv_tolerance, self.srv_time_out)

            # move to erroneous wrist pose
            wrist_init_pose = self.get_wrist_init_pose(truth_wrist_t, truth_wrist_q, test_case.wrist_error)
            res = self.cmd_wrist_abs(wrist_init_pose, True, True)
            if not res:
                rospy.logwarn("Could not reach erroneous wrist pose. Resetting again.")
                self.resetting_attempts += 1
                return self.reset_world(state, test_case)

            # close fingers
            self.close_until_contact_and_tighten()
            self.wait_until_grasp_stabilizes()
            self.start_obj_t, start_obj_q = get_tq_from_homo_matrix(self.get_object_pose())

            # check if we knocked object over
            start_obj_q_euler = np.array(tf.transformations.euler_from_quaternion(start_obj_q))
            obj_q_euler = np.array(tf.transformations.euler_from_quaternion(obj_q))
            trans = np.linalg.norm((np.array(self.start_obj_t) - np.array(obj_t)))
            rot = np.linalg.norm((start_obj_q_euler - obj_q_euler), ord=1)
            large_translation = trans > self.hparams["z_error_max"] * 1.5
            large_rot = rot > deg2rad(80)
            if large_translation or large_rot:
                rospy.logerr(
                    f"Object knocked over or rolled away while resetting. Translation: {trans}-{large_translation}, Rotation: {rot}-{large_rot}. Resetting again!"
                )
                self.knocked_obj_over = True
                return self.reset_world(state, test_case)

        except Exception as e:
            rospy.logerr(f"Exception occurred while resetting: '{e}'. Resetting again")
            rospy.sleep(2)
            self.resetting_attempts += 1
            return self.reset_world(state, test_case)

        self.resetting_attempts = 0

    def close_until_contact_and_tighten(self, tighten_incr=0):
        res = self.close_until_contact(TriggerRequest())
        if self.verbose:
            rospy.loginfo("Closed reflex fingers until contact: \n" + str(res))
        if tighten_incr:
            res = self.finger_pos_incr([tighten_incr, tighten_incr, tighten_incr, 0])
            if self.verbose:
                rospy.loginfo("Tightened fingers by " + str(tighten_incr) + ": \n" + str(res))

    def object_lifted(self):
        msg = rospy.wait_for_message("/gazebo/object_sensor_bumper", ContactsState)
        collision_name = "ground_plane::link::collision"

        for i in range(len(msg.states)):
            if msg.states[i].collision1_name == collision_name or msg.states[i].collision2_name == collision_name:
                return False
        return True

    def get_wrist_init_pose(self, wrist_p, wrist_q, wrist_error):

        rospy.loginfo(
            f"Random offset for init wrist pose is \n [x: {wrist_error.x}, y: {wrist_error.y}, z: {wrist_error.z}], [roll: {wrist_error.roll}, pitch: {wrist_error.pitch}, yaw: {wrist_error.yaw}]."
        )

        # construct matrix that offsets from ground truth pose
        q = tf.transformations.quaternion_from_euler(wrist_error.roll, wrist_error.pitch, wrist_error.yaw)
        mat_t_offset = tf.transformations.translation_matrix([wrist_error.x, wrist_error.y, wrist_error.z])
        mat_q_offset = tf.transformations.quaternion_matrix(q)
        mat_offset = np.dot(mat_t_offset, mat_q_offset)

        truth_wrist_init_pose = get_homo_matrix_from_tq(wrist_p, wrist_q)
        return np.dot(truth_wrist_init_pose, mat_offset)
