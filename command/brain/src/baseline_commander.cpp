#include <math.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/Hand.h>
#include <gazebo_msgs/GetModelState.h>

using namespace std;

string node_name = "baseline_commander_node";
string source_frame = "world";
string target_frame = "reflex";

string sph_open_srv_name = "reflex/spherical_open";
string sph_close_srv_name = "reflex/spherical_close";
string state_topic_name = "reflex/hand_state";

class FingerState
{
private:
    int finger_id;
    int num_sensors = 9;
    array<bool, 9> sensor_contacts;
    array<float, 9> pressure;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;

public:
    FingerState(int finger_id)
    {
        this->finger_id = finger_id;
    }

    void setProximalFromMsg(float proximal_angle)
    {
        this->proximal_angle = proximal_angle;
    }

    void setDistalFromMsg(float distal_angle)
    {
        this->distal_angle = distal_angle;
    }

    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts)
    {
        for (int i = 0; i < num_sensors; i++)
        {
            this->sensor_contacts[i] = sensor_contacts[i];
        }
    }

    void setPressureFromMsg(boost::array<float, 9> pressure)
    {
        for (int i = 0; i < num_sensors; i++)
        {
            this->pressure[i] = pressure[i];
        }
    }

    bool hasContact()
    {
        for (int i = 0; i < num_sensors; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }

    bool hasProximalContact()
    {
        for (int i = 0; i < 5; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }

    bool hasDistalContact()
    {
        for (int i = 5; i < num_sensors; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }
};

class HandState
{
public:
    enum State
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    State cur_state;
    int num_fingers = 3;
    int num_motors = 4;

    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

    void updateState()
    {
        int contact_count = countFingersInContact();
        switch (contact_count)
        {
        case 0:
            cur_state = NoContact;
            break;
        case 1:
            cur_state = SingleFingerContact;
            break;
        default:
            cur_state = MultipleFingerContact;
            break;
        }
    }

    State getCurrentState()
    {
        updateState();
        return cur_state;
    }

    int countFingersInContact()
    {
        int count = 0;
        for (int i = 0; i < num_fingers; i++)
        {
            if (finger_states[i].hasContact() == true)
            {
                count++;
            }
        }
        return count;
    }
};

class BaselineCommander
{
private:
    std_srvs::Trigger trigger;
    ros::ServiceClient sph_open_client;
    ros::ServiceClient sph_close_client;
    ros::Subscriber reflex_state_sub;

    // transform broadcaster
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform cur_transform;
    tf2::Transform goal_transform;

    float backoff_factor = 1.0;
    float step_size = 0.001;
    bool grasped = false;
    HandState hand_state = HandState();

public:
    BaselineCommander(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose)
    {
        sph_open_client = nh->serviceClient<std_srvs::Trigger>(sph_open_srv_name);
        sph_close_client = nh->serviceClient<std_srvs::Trigger>(sph_close_srv_name);
        reflex_state_sub = nh->subscribe(state_topic_name, 1, &BaselineCommander::callbackHandState, this);
        goal_transform = goal_wrist_pose;

        // TODO find another, more elegant solution for this
        // wait before publishing first transform to fix warning from wrist_controller_node
        // ""reflex" passed to lookupTransform argument target_frame does not exist."
        ros::Duration(1).sleep();

        cur_transform = init_wrist_pose;
        sendTransform(cur_transform);

        // put fingers in spherical open position
        sph_open_client.call(trigger);
        ROS_INFO("%s", trigger.response.message.c_str());

        // wait before first time_step call
        ros::Duration(0.5).sleep();
    }

    void callbackHandState(const reflex_msgs::Hand &msg)
    {
        for (int i = 0; i < hand_state.num_fingers; i++)
        {
            hand_state.finger_states[i].setProximalFromMsg(msg.finger[i].proximal);
            hand_state.finger_states[i].setDistalFromMsg(msg.finger[i].distal_approx);
            hand_state.finger_states[i].setSensorContactsFromMsg(msg.finger[i].contact);
            hand_state.finger_states[i].setPressureFromMsg(msg.finger[i].pressure);
        }
    }

    // moves along vector in reflex coordinates
    // length of vector is step size
    void moveAlongVector(tf2::Vector3 vector)
    {
        tf2::Transform increment = tf2::Transform();
        increment.setIdentity();
        increment.setOrigin(vector);
        cur_transform *= increment;

        sendTransform(cur_transform);
    }

    void sendTransform(tf2::Transform transform)
    {
        ts.header.frame_id = source_frame;
        ts.child_frame_id = target_frame;
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(transform);
        br.sendTransform(ts);
    }

    void timeStep()
    {
        if (grasped == false)
        {
            // check if we can make grasping attempt
            switch (hand_state.getCurrentState())
            {
            case HandState::State::NoContact:
            {
                ROS_INFO("No contact --> Approach");
                tf2::Vector3 reflex_z_increment = {0, 0, step_size};
                moveAlongVector(reflex_z_increment);
                break;
            }
            case HandState::State::SingleFingerContact:
            {
                // TODO update approach direction
                ROS_INFO("Single contact --> Approach");
                tf2::Vector3 reflex_z_increment = {0, 0, step_size};
                moveAlongVector(reflex_z_increment);
                break;
            }
            case HandState::State::MultipleFingerContact:
            {
                // do spherical grasp
                ROS_INFO("Multi contact --> Grasping");
                sph_close_client.call(trigger);
                grasped = true;
                ros::Duration(0.5).sleep();
                break;
            }
            }
        }
        else
        {
            // we grasped, move to goal pose
            ROS_INFO("Grasped object --> Moving to goal pose");
            sendTransform(goal_transform);
        }
    }
};

tf2::Transform calcInitWristPose(ros::NodeHandle *nh,
                            tf2::Vector3 pos_error = {0, 0, 0},
                            float polar = M_PI/4,
                            float azimuth = 0)
{
    ROS_INFO("Calculating initial wrist pose.");
    ///////////////////////////////////////////////////////
    // A) GET GROUND TRUTH OBJECT POSE IN WORLD COORDINATES
    ///////////////////////////////////////////////////////

    // get object name from parameter server
    // TODO: make a generic function out of this -> with template
    string object_name;
    nh->getParam("object_name", object_name);

    // setup service client
    string service_name = "/gazebo/get_model_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::GetModelState>(service_name);

    // setup service message
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = object_name;
    srv.request.relative_entity_name = "world";

    // obtain position of object (we don't care about its orientation for now)
    client.call(srv);
    tf2::Vector3 t = {srv.response.pose.position.x,
                      srv.response.pose.position.y,
                      srv.response.pose.position.z};

    ROS_INFO("Obtained object pose in world coordinates.");
    ROS_INFO_STREAM("Object position: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);

    ////////////////////////////////////////////////////////////
    // B) INTRODUCE ERRORS TO OBJECT POSITION TO TEST ROBUSTNESS
    ////////////////////////////////////////////////////////////

    // NOTE: for now not introducing orientation errors because we are currently
    // basing the algorithm only on object position. This is also why we only translate
    // but do not rotate from world to object frame. 
    // TODO: get error from param server
    t += pos_error;
    tf2::Transform translate_to_object(tf2::Quaternion {0, 0, 0, 1}, t);

    /////////////////////////////////////////////////////////////////////
    // C) ROTATE AND TRANSLATE IN SPHERICAL COORDINATES TO GET WRIST POSE
    /////////////////////////////////////////////////////////////////////

    // add M_PI to polar angle, s.t. Reflex z points in opposite direction of 
    polar += M_PI;

    // rotation with spherical coordinates
    tf2::Quaternion q;
    q.setRPY(0, polar, azimuth);
    tf2::Transform rotate_spherical;
    rotate_spherical.setRotation(q);

    // translation to tool center point (TCP) along negative z axis
    tf2::Vector3 tcp_to_object_offset = tf2::Vector3{0, 0, -0.3};
    tf2::Transform translate_to_tcp = tf2::Transform();
    translate_to_tcp.setIdentity();
    translate_to_tcp.setOrigin(tcp_to_object_offset);

    // translate to flange frame (values taken from Reflex CAD drawing available on website)
    // z_offset: distance from Reflex origin to palm surface
    // x_offset: approx. distance along x axis from origin to palm center
    float reflex_height = -0.09228;
    float x_offset = -0.015;
    tf2::Transform translate_to_wrist = tf2::Transform();
    translate_to_wrist.setIdentity();
    translate_to_wrist.setOrigin(tf2::Vector3{x_offset, 0, reflex_height});

    tf2::Transform init_wrist_pose = translate_to_object * rotate_spherical * translate_to_tcp * translate_to_wrist;

    // debug information
    tf2::Vector3 init_t = init_wrist_pose.getOrigin();
    tf2::Quaternion init_r = init_wrist_pose.getRotation();
    ROS_INFO("Done calculating initial wrist pose.");
    ROS_INFO_STREAM("Initial wrist position: x=" << init_t[0] << ", y=" << init_t[1] << ", z=" << init_t[2]);
    ROS_INFO_STREAM("Initial wrist orientation: x=" << init_r[0] << ", y=" << init_r[1] << ", z=" << init_r[2] << ", w=" << init_r[3]);

    return init_wrist_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(40);
    ROS_INFO("Launched %s.", node_name.c_str());

    tf2::Transform init_wrist_pose;
    bool simulation_only;
    string desired_param = "simulation_only";

    // wait for simulation_only on parameter server
    while (ros::ok())
    {
        if (nh.hasParam(desired_param))
        {
            nh.getParam(desired_param, simulation_only);
            ROS_INFO("Obtained %s: '%s' from parameter server.", desired_param.c_str(), simulation_only ? "1" : "0");
            break;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", desired_param.c_str());
            ros::Duration(1.0).sleep();
        }
    }

    if (simulation_only == true)
    {
        // calculate initial wrist pose s.t. hand is in good grasping pose
        init_wrist_pose = calcInitWristPose(&nh);
    }
    else
    {
        // obtain initial pose from robot arm or from computer vision system
        // TODO: wait until we receive the initial pose (e.g. through a rosservice)
        ROS_WARN("Not implemented.");
    }

    // goal pose: goal_position and facing downwards.
    tf2::Vector3 goal_position = tf2::Vector3{0.3, 0, 0.3};
    tf2::Quaternion goal_rotation;
    goal_rotation.setRPY(M_PI, 0, 0);
    tf2::Transform goal_wrist_pose = tf2::Transform(goal_rotation, goal_position);

    BaselineCommander bc = BaselineCommander(&nh, init_wrist_pose, goal_wrist_pose);

    ros::Duration(1.0).sleep();
    ROS_INFO("Starting autonomous control.");

    while (ros::ok())
    {
        bc.timeStep();
        ros::spinOnce();
        rate.sleep();
    }
}