#ifndef HAND_COMMAND_H
#define HAND_COMMAND_H

#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/PoseCommand.h>

#include "reflex_interface/PosIncrement.h"
#include "reflex_interface/GraspPrimitive.h"
#include "reflex_interface/hand_state.hpp"

class HandCommand
{
public:
    HandCommand(ros::NodeHandle *nh, HandState *state, bool use_sim_data_hand);

    enum Primitive
    {
        Open,
        Close,
        Pinch,
        SphericalOpen,
        SphericalClose
    };
    bool executePrimitive(Primitive primitive, bool blocking = false, float tolerance = 0.1, float time_out = 4, std::string *status_msg = nullptr);

private:
    HandState *state;

    std::string open_srv_name = "reflex_interface/open";
    std::string close_srv_name = "reflex_interface/close";
    std::string pinch_srv_name = "reflex_interface/pinch";
    std::string sph_open_srv_name = "reflex_interface/spherical_open";
    std::string sph_close_srv_name = "reflex_interface/spherical_close";
    std::string pos_incr_srv_name = "reflex_interface/position_increment";
    std::string close_until_contact_srv_name = "reflex_interface/close_until_contact";
    std::string tighten_grip_srv_name = "reflex_interface/tighten_grip";
    std::string pos_cmd_topic = "reflex_takktile/command_position";

    float tighten_incr = 0.0349066;    // this is 2 degrees
    float close_until_contact_incr = 0.1;
    float close_until_contact_time_out = 6;
    float close_until_contact_pub_rate = 50;

    ros::Publisher pub;
    ros::ServiceServer open_service;
    ros::ServiceServer close_service;
    ros::ServiceServer pinch_service;
    ros::ServiceServer sph_open_service;
    ros::ServiceServer sph_close_service;
    ros::ServiceServer pos_incr_service;
    ros::ServiceServer close_until_contact_service;
    ros::ServiceServer tighten_grip_service;
    reflex_msgs::PoseCommand pos_cmd;

    // format {finger1, finger2, finger3, preshape}
    std::array<float, 4> low_limits = {0, 0, 0, 0};
    std::array<float, 4> high_limits = {M_PI, M_PI, M_PI, M_PI};
    std::array<float, 4> open_pos = {0, 0, 0, 0};
    std::array<float, 4> close_pos = {2.5, 2.5, 2.5, 0};
    std::array<float, 4> pinch_pos = {1.7, 1.7, 0, M_PI};
    std::array<float, 4> sph_open_pos = {1.2, 1.2, 1.2, M_PI / 2};
    std::array<float, 4> sph_close_pos = {2.5, 2.5, 2.5, M_PI / 2};
    std::array<float, 4> cur_cmd = open_pos;

    std::string getStatusMsg();

    bool callbackOpen(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res);
    bool callbackClose(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res);
    bool callbackPinch(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res);
    bool callbackSphOpen(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res);
    bool callbackSphClose(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res);
    bool callbackPosIncr(reflex_interface::PosIncrement::Request &req, reflex_interface::PosIncrement::Response &res);
    bool callbackCloseUntilContact(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackTightenGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool executePosIncrement(float increment[4], bool from_measured_pos = true, bool blocking = false, float tolerance = 0.1, float time_out = 4);
    void publishCommand();
    bool waitUntilFinished(float tolerance, float time_out);
};

#endif