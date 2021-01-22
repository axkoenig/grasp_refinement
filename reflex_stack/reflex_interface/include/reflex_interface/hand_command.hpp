#ifndef HAND_COMMAND_H
#define HAND_COMMAND_H

#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/PoseCommand.h>

#include "reflex_interface/PosIncrement.h"

class HandCommand
{
public:
    HandCommand(ros::NodeHandle *nh);

    enum Primitive
    {
        Open,
        Close,
        Pinch,
        SphericalOpen,
        SphericalClose
    };
    void executePrimitive(Primitive primitive, bool verbose = true);

private:
    std::string open_srv_name = "reflex/open";
    std::string close_srv_name = "reflex/close";
    std::string pinch_srv_name = "reflex/pinch";
    std::string sph_open_srv_name = "reflex/spherical_open";
    std::string sph_close_srv_name = "reflex/spherical_close";
    std::string pos_incr_srv_name = "reflex/pos_incr";
    std::string pos_cmd_topic = "reflex/pos_cmd";

    ros::Publisher pub;
    ros::ServiceServer open_service;
    ros::ServiceServer close_service;
    ros::ServiceServer pinch_service;
    ros::ServiceServer sph_open_service;
    ros::ServiceServer sph_close_service;
    ros::ServiceServer pos_incr_service;
    reflex_msgs::PoseCommand pos_cmd;

    // format {finger1, finger2, finger3, preshape}
    std::array<float, 4> low_limits = {0, 0, 0, 0};
    std::array<float, 4> high_limits = {M_PI, M_PI, M_PI, M_PI};
    std::array<float, 4> open_pos = {0, 0, 0, 0};
    std::array<float, 4> close_pos = {2.5, 2.5, 2.5, 0};
    std::array<float, 4> pinch_pos = {1.7, 1.7, 0, M_PI};
    std::array<float, 4> sph_open_pos = {1.2, 1.2, 1.2, M_PI / 2};
    std::array<float, 4> sph_close_pos = {2.5, 2.5, 2.5, M_PI / 2};
    std::array<float, 4> cur_pos = open_pos;

    std::string getStatusMsg();
    bool callbackOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackPinch(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackSphOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackSphClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool callbackPosIncr(reflex_interface::PosIncrement::Request &req, reflex_interface::PosIncrement::Response &res);
    void executePosIncrement(float increment[4]);
    void sendCommands();
};

#endif