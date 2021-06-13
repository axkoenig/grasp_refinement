import random
import os
import subprocess
from subprocess import DEVNULL

import numpy as np
import rospy
import roslib
import tf2_ros
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
from .tests import RandomCylinder, RandomBox


class GazeboInterface:
    def __init__(self, verbose=True):
        self.verbose = verbose

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
            self.wait_until_reached_pose(mat_shell)

    def cmd_wrist_pose_incr(self, p_incr, q_incr, wait_until_reached_pose=False):
        mat_t_incr = tf.transformations.translation_matrix(p_incr)
        mat_q_incr = tf.transformations.quaternion_matrix(q_incr)
        mat_homo = np.dot(mat_t_incr, mat_q_incr)
        self.last_wrist_pose = np.dot(self.last_wrist_pose, mat_homo)
        self.send_transform(self.last_wrist_pose)
        if wait_until_reached_pose:
            self.wait_until_reached_pose(self.last_wrist_pose)

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
                return
            r.sleep()
        rospy.loginfo(f"Did not reach pose in time out of {time_out} secs. You're probably crashing into something ...")

    def wait_until_grasp_stabilizes(self):
        rospy.sleep(0.2)

    def delete_model(self, name="object"):
        req = DeleteModelRequest()
        req.model_name = name
        service_call_with_retries(self.delete_model, req)

    def get_cur_obj_name(self):
        msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        models = msg.name
        irrelevant_objects = ["ground_plane", "reflex"]
        for obj in irrelevant_objects:
            if obj in models:
                models.remove(obj)
        if len(models) != 1:
            rospy.logwarn("You have more or less than one model in your world! Returning '' for object name.")
            return ""
        return models[0]

    def kill_controllers(self):
        nodes = [
            "gazebo/finger_controller_spawner",
            "gazebo/wrist_controller_spawner",
        ]
        if self.verbose:
            rospy.loginfo("Killing controller nodes")
        for node in nodes:
            os.system("rosnode kill " + node)

    def launch_random_object(self):
        object = random.choice(["cylinder", "box"])
        self.launch_cylinder(RandomCylinder()) if object == "cylinder" else self.launch_box(RandomBox())
        rospy.sleep(1)

    def launch_cylinder(self, cylinder):
        rospy.loginfo(f"Spawning cylinder {cylinder.name} ...")
        os.system(
            f"roslaunch description object.launch object_name:=cylinder object_spawn_name:={cylinder.name} cylinder_radius:={cylinder.radius} cylinder_length:={cylinder.length} inertia_scaling_factor:={cylinder.inertia_scaling_factor}"
        )
        rospy.set_param("object_name", cylinder.name)

    def launch_box(self, box):
        rospy.loginfo(f"Spawning box {box.name}...")
        os.system(
            f"roslaunch description object.launch object_name:=box object_spawn_name:={box.name} box_x:={box.x} box_y:={box.y} box_z:={box.z} inertia_scaling_factor:={box.inertia_scaling_factor}"
        )
        rospy.set_param("object_name", box.name)

    def run_cmd_in_subprocess(self, cmd):
        subprocess.Popen([cmd], shell=True, stdout=DEVNULL)

    def launch_test_obj(self, test_case):
        rospy.loginfo("Launching object from test case.")
        self.launch_cylinder(test_case.object) if test_case.object_name == "cylinder" else self.launch_box(test_case.object)
        rospy.sleep(1)

    def reset_world(self, hparams, test_case=None):
        self.kill_controllers()
        self.delete_model("reflex")
        rospy.sleep(0.5)

        # delete old object and relaunch a new object
        self.object_name = self.get_cur_obj_name()
        self.delete_model(self.object_name)
        rospy.sleep(0.5)
        self.launch_random_object() if not test_case else self.launch_test_obj(test_case)

        # reset reflex pose and spawn new reflex
        self.cmd_wrist_abs(tf.transformations.identity_matrix(), False)
        self.spawn_reflex()
        self.spawn_controllers()

        # get ground truth pose of reflex (which is offset from object)
        obj_t, _ = get_tq_from_homo_matrix(self.get_object_pose())
        truth_wrist_t = obj_t - [0, 0.05, 0]  # 5cm offset
        truth_wrist_q = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, 0)

        # move to waypoint poses
        wrist_waypoint_pose = get_homo_matrix_from_tq([0, 0, 0.12], truth_wrist_q)
        self.cmd_wrist_abs(wrist_waypoint_pose, True, True)
        self.open_hand(True, self.srv_tolerance, self.srv_time_out)

        # move to erroneous wrist pose
        wrist_init_pose = self.get_wrist_init_pose(truth_wrist_t, truth_wrist_q, hparams, test_case)
        self.cmd_wrist_abs(wrist_init_pose, True, True)

        # close fingers
        self.close_until_contact_and_tighten()
        self.wait_until_grasp_stabilizes()
        self.start_obj_t, _ = get_tq_from_homo_matrix(self.get_object_pose())

    def spawn_controllers(self):
        rospy.loginfo("Spawning reflex controllers ...")
        cmd = ["roslaunch finger_controller finger_controller.launch only_spawn:=true"]
        subprocess.Popen(cmd, shell=True, stdout=DEVNULL)
        rospy.sleep(2)
        rospy.loginfo("Spawning wrist controllers ...")
        cmd = ["roslaunch wrist_controller wrist_controller.launch only_spawn:=true"]
        subprocess.Popen(cmd, shell=True, stdout=DEVNULL)
        rospy.sleep(2)

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

    def get_wrist_init_pose(self, wrist_p, wrist_q, hparams, test_case=None):

        if test_case is None:
            # generate random offset from initial wrist pose
            x = np.random.uniform(hparams["x_error_min"], hparams["x_error_max"])
            y = np.random.uniform(hparams["y_error_min"], hparams["y_error_max"])
            z = np.random.uniform(hparams["z_error_min"], hparams["z_error_max"])
            roll = np.random.uniform(hparams["roll_error_min"], hparams["roll_error_max"])
            pitch = np.random.uniform(hparams["pitch_error_min"], hparams["pitch_error_max"])
            yaw = np.random.uniform(hparams["yaw_error_min"], hparams["yaw_error_max"])
        else:
            rospy.loginfo("Loading wrist error from test case")
            x, y, z, roll, pitch, yaw = test_case.get_wrist_error()

        rospy.loginfo(f"Random offset for init wrist pose is \n [x: {x}, y: {y}, z: {z}], [roll: {roll}, pitch: {pitch}, yaw: {yaw}].")

        # construct matrix that offsets from ground truth pose
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        mat_t_offset = tf.transformations.translation_matrix([x, y, z])
        mat_q_offset = tf.transformations.quaternion_matrix(q)
        mat_offset = np.dot(mat_t_offset, mat_q_offset)

        truth_wrist_init_pose = get_homo_matrix_from_tq(wrist_p, wrist_q)
        return np.dot(truth_wrist_init_pose, mat_offset)

    def spawn_object(self):
        req = SpawnModelRequest()
        req.model_name = self.new_obj_name
        urdf_location = roslib.packages.get_pkg_dir("description") + f"/urdf/objects/{self.new_obj_name}.urdf"
        req.model_xml = open(urdf_location, "r").read()
        req.reference_frame = "world"
        req.initial_pose = Pose(Point(self.obj_t[0], self.obj_t[1], self.obj_t[2]), Quaternion(self.obj_q[0], self.obj_q[1], self.obj_q[2], self.obj_q[3]))
        res = service_call_with_retries(self.spawn_sdf_model, req)

        if res.success:
            rospy.set_param("object_name", self.new_obj_name)

    def spawn_reflex(self):
        # read xacro file
        urdf_location = roslib.packages.get_pkg_dir("description") + f"/robots/reflex.robot.xacro"
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

        if not res.success:
            rospy.logfatal("Failed to spawn reflex.")