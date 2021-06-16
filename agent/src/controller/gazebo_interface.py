import random
import os
import subprocess
from subprocess import DEVNULL

import numpy as np
import rospy
import roslib
import tf2_ros
import roslaunch
import tf

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_srvs.srv import Empty, Trigger, TriggerRequest
from gazebo_msgs.srv import GetModelState, GetLinkState, SetModelState, DeleteModel, SpawnModel, DeleteModelRequest, SpawnModelRequest
from gazebo_msgs.msg import ModelState, ModelStates, ContactsState

from reflex_interface.srv import PosIncrement, GraspPrimitive
from .helpers.transforms import (
    get_tq_from_homo_matrix,
    get_homo_matrix_from_tq,
    get_homo_matrix_from_pose_msg,
)
from .helpers.services import (
    StringServiceRequest,
    service_call_with_retries,
)
from .tests import gen_valid_wrist_error_obj_combination_from_ranges


class GazeboInterface:
    def __init__(self, hparams, verbose=True):
        self.verbose = verbose
        self.hparams = hparams
        self.description_path = roslib.packages.get_pkg_dir("description")
        self.object_name = self.get_ros_param_with_retries("/object_name")
        self.object_names = self.get_ros_param_with_retries("/object_names")

        # gazebo services
        rospy.wait_for_service("/gazebo/unpause_physics")
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.spawn_urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

        # reflex services
        rospy.wait_for_service("/reflex_interface/open")
        self.open_hand = rospy.ServiceProxy("/reflex_interface/open", GraspPrimitive)
        self.close_until_contact = rospy.ServiceProxy("/reflex_interface/close_until_contact", Trigger)
        self.pos_incr = rospy.ServiceProxy("/reflex_interface/position_increment", PosIncrement)

        # some vars
        self.num_srv_tries = 0
        self.srv_time_out = 5
        self.srv_tolerance = 0.01

        # get some info on reflex setup
        self.simplify_collisions = rospy.get_param("simplify_collisions")
        self.simplify_collisions = "true" if self.simplify_collisions else "false"
        self.reflex_pub_rate = rospy.get_param("reflex_pub_rate")

        # setup broadcaster for desired wrist pose
        self.ts_wrist = TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.sim_unpause()  # make sure simulation is not paused
        rospy.sleep(1)  # broadcaster needs some time to start

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch_controllers()

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
        self.object_name = self.get_cur_obj_name()
        req = StringServiceRequest(self.object_name, "world")
        res = service_call_with_retries(self.get_model_state, req)
        if not res:  # sometimes we don't get a result (we handle this during resetting)
            rospy.logwarn("Could not get object pose! Returning identity")
            return tf.transformations.identity_matrix()
        return get_homo_matrix_from_pose_msg(res.pose)

    def get_wrist_pose(self):
        req = StringServiceRequest("shell", "world")
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

    def cmd_wrist_pose_incr(self, p_incr, q_incr, wait_until_reached_pose=False):
        mat_t_incr = tf.transformations.translation_matrix(p_incr)
        mat_q_incr = tf.transformations.quaternion_matrix(q_incr)
        mat_homo = np.dot(mat_t_incr, mat_q_incr)
        self.last_wrist_pose = np.dot(self.last_wrist_pose, mat_homo)
        self.send_transform(self.last_wrist_pose)
        if wait_until_reached_pose:
            return self.wait_until_reached_pose(self.last_wrist_pose)
        return 1

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
        rospy.sleep(0.2)

    def delete_model(self, name="object"):
        req = DeleteModelRequest()
        req.model_name = name
        service_call_with_retries(self.delete_model, req)

    def get_cur_obj_name(self):
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        models = msg.name
        rospy.loginfo("Models in world are: " + str(models))
        irrelevant_objects = ["ground_plane", "reflex"]
        for obj in irrelevant_objects:
            if obj in models:
                models.remove(obj)
        if len(models) != 1:
            rospy.logwarn("You have more or less than one model in your world! Returning '' for object name.")
            return ""
        return models[0]

    def shutdown_controllers(self):
        rospy.loginfo("Shutting down reflex controller nodes")
        self.finger_ctrl_launch.shutdown()
        self.wrist_ctrl_launch.shutdown()

    def launch_object(self, object):
        launch_file = self.description_path + "/launch/object.launch"
        if object.__class__.__name__ == "RandomCylinder":
            cli_args = [
                launch_file,
                "object_name:=cylinder",
                f"object_spawn_name:={object.name}",
                f"cylinder_radius:={object.radius}",
                f"cylinder_length:={object.length}",
                f"inertia_scaling_factor:={object.inertia_scaling_factor}",
            ]
        elif object.__class__.__name__ == "RandomBox":
            cli_args = [
                launch_file,
                "object_name:=box",
                f"object_spawn_name:={object.name}",
                f"box_x:={object.x}",
                f"box_y:={object.y}",
                f"box_z:={object.z}",
                f"inertia_scaling_factor:={object.inertia_scaling_factor}",
            ]
        else:
            rospy.logerr("Unsupported object type!")
            return
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        parent = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        parent.start()
        rospy.set_param("object_name", object.name)

    def delete_all_models(self):
        # deletes all models except ground plane
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        models = msg.name
        for model in models:
            if model != "ground_plane":
                self.delete_model(model)
                rospy.sleep(0.5)

    def reset_world(self, test_case=None):
        try:
            self.shutdown_controllers()
            self.delete_all_models()

            # reset reflex pose and spawn new reflex
            self.cmd_wrist_abs(tf.transformations.identity_matrix())
            res = self.spawn_reflex()
            if not res:
                rospy.logwarn("Could not spawn reflex. Resetting again.")
                return self.reset_world(test_case)
            rospy.sleep(2)
            self.launch_controllers()

            # launch new object
            if not test_case:  # we're training
                object_type = random.choice(["cylinder", "box"])
                object, wrist_error = gen_valid_wrist_error_obj_combination_from_ranges(object_type, self.hparams)
                res = self.spawn_object(object)
            else:  # we're testing
                wrist_error = test_case.wrist_error
                res = self.spawn_object(test_case.object)
            if not res:
                rospy.logwarn("Could not spawn object. Resetting again.")
                return self.reset_world(test_case)
            rospy.sleep(2)  # required s.t. object can register with gazebo

            obj_pose = self.get_object_pose()
            if (obj_pose == tf.transformations.identity_matrix()).all():
                rospy.logwarn("Could not get object pose. Resetting again.")
                return self.reset_world(test_case)

            # get ground truth pose of reflex (which is offset from object frame)
            obj_t, _ = get_tq_from_homo_matrix(obj_pose)
            truth_wrist_t = obj_t - [0, 0.05, 0]  # 5cm offset
            truth_wrist_q = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, 0)

            # move to waypoint poses
            wrist_waypoint_pose = get_homo_matrix_from_tq([0, 0, 0.12], truth_wrist_q)
            res = self.cmd_wrist_abs(wrist_waypoint_pose, True, True)
            if not res:
                rospy.logwarn("Could not reach waypoint wrist pose. Resetting again.")
                return self.reset_world(test_case)
            self.open_hand(True, self.srv_tolerance, self.srv_time_out)

            # move to erroneous wrist pose
            wrist_init_pose = self.get_wrist_init_pose(truth_wrist_t, truth_wrist_q, wrist_error)
            res = self.cmd_wrist_abs(wrist_init_pose, True, True)
            if not res:
                rospy.logwarn("Could not reach erroneous wrist pose. Resetting again.")
                return self.reset_world(test_case)

            # close fingers
            self.close_until_contact_and_tighten()
            self.wait_until_grasp_stabilizes()
            self.start_obj_t, _ = get_tq_from_homo_matrix(self.get_object_pose())
        except Exception as e:
            # TODO delete this once we found the problem!
            rospy.logerr(f"Exception occurred while resetting: '{e}'. Resetting again")
            rospy.sleep(2)
            return self.reset_world(test_case)

    def launch_controllers(self):
        rospy.loginfo("Launching finger controllers ...")
        launch_file = roslib.packages.get_pkg_dir("finger_controller") + "/launch/finger_controller.launch"
        cli_args = [launch_file, "only_spawn:=true"]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.finger_ctrl_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        self.finger_ctrl_launch.start()

        rospy.loginfo("Launching wrist controllers ...")
        launch_file = roslib.packages.get_pkg_dir("wrist_controller") + "/launch/wrist_controller.launch"
        cli_args = [launch_file, "only_spawn:=true"]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.wrist_ctrl_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        self.wrist_ctrl_launch.start()

    def close_until_contact_and_tighten(self, tighten_incr=0):
        res = self.close_until_contact(TriggerRequest())
        if self.verbose:
            rospy.loginfo("Closed reflex fingers until contact: \n" + str(res))
        if tighten_incr:
            res = self.pos_incr(tighten_incr, tighten_incr, tighten_incr, 0, False, False, 0, 0)
            if self.verbose:
                rospy.loginfo("Tightened fingers by " + str(tighten_incr) + ": \n" + str(res))

    def intelligent_reopen(self, prox_angles, back_off=-0.3, min_angle=1):
        # calc back off to guarantee a minimum reopening angle (otherwise a finger may get stuck in a closed position)
        back_offs = [0, 0, 0]
        for i in range(3):
            if prox_angles[i] + back_off > min_angle:
                back_offs[i] = min_angle - prox_angles[i]
            else:
                back_offs[i] = back_off
        if self.verbose:
            rospy.loginfo("Backing fingers off by: \t" + str(back_offs))
        res = self.pos_incr(back_offs[0], back_offs[1], back_offs[2], 0, True, True, self.srv_tolerance, 1)
        if self.verbose:
            rospy.loginfo("Backed off fingers: \n" + str(res))

    def regrasp(self, wrist_p_incr, wrist_q_incr, prox_angles):

        self.intelligent_reopen(prox_angles)
        self.cmd_wrist_pose_incr(wrist_p_incr, wrist_q_incr)
        self.close_until_contact_and_tighten()
        self.wait_until_grasp_stabilizes()

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

    def spawn_object(self, object):
        if object.__class__.__name__ == "RandomCylinder":
            urdf_location = self.description_path + f"/urdf/environment/cylinder.urdf.xacro"
            p = os.popen(
                "xacro "
                + urdf_location
                + f" cylinder_radius:={object.radius} cylinder_length:={object.length} inertia_scaling_factor:={object.inertia_scaling_factor}"
            )
            height = object.length
        elif object.__class__.__name__ == "RandomBox":
            urdf_location = self.description_path + f"/urdf/environment/box.urdf.xacro"
            p = os.popen(
                "xacro " + urdf_location + f" box_x:={object.x} box_y:={object.y} box_z:={object.z} inertia_scaling_factor:={object.inertia_scaling_factor}"
            )
            height = object.z
        else:
            rospy.logerr("Unsupported object type!")
            return

        xml_string = p.read()
        p.close()

        req = SpawnModelRequest()
        req.model_name = object.name
        req.model_xml = xml_string
        req.reference_frame = "world"
        req.initial_pose = Pose(Point(0, 0.2, height / 2 + 0.01), Quaternion(0, 0, 0, 1))
        res = service_call_with_retries(self.spawn_sdf_model, req)

        if res is None:
            rospy.logerr("Failed to spawn object. Did not get result from service.")
            return 0
        if not res.success:
            rospy.logerr("Failed to spawn object. Result of service is false.")
            return 0

        rospy.set_param("object_name", object.name)
        return 1

    def spawn_reflex(self):
        # read xacro file
        urdf_location = self.description_path + f"/robots/reflex.robot.xacro"
        p = os.popen(
            "xacro " + urdf_location + f" base_link_name:=shell reflex_pub_rate:={self.reflex_pub_rate} simplify_collisions:={self.simplify_collisions}"
        )
        xml_string = p.read()
        p.close()

        req = SpawnModelRequest()
        req.model_name = "reflex"
        req.model_xml = xml_string
        req.reference_frame = "world"
        res = service_call_with_retries(self.spawn_sdf_model, req)

        if res is None:
            rospy.logerr("Failed to spawn reflex. Did not get result from service.")
            return 0
        if not res.success:
            rospy.logerr("Failed to spawn reflex. Result of service is false.")
            return 0

        return 1