#include <algorithm>
#include <bits/stdc++.h>
#include <mutex>

#include <std_srvs/Empty.h>

#include "reflex_interface/PosIncrement.h"
#include "reflex_interface/hand_command.hpp"

HandCommand::HandCommand(ros::NodeHandle *nh, HandState *state, bool use_sim_data_hand)
{
    this->state = state;

    pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
    open_service = nh->advertiseService(open_srv_name, &HandCommand::callbackOpen, this);
    close_service = nh->advertiseService(close_srv_name, &HandCommand::callbackClose, this);
    sph_open_service = nh->advertiseService(sph_open_srv_name, &HandCommand::callbackSphOpen, this);
    pos_incr_service = nh->advertiseService(pos_incr_srv_name, &HandCommand::callbackPosIncr, this);
    close_until_contact_service = nh->advertiseService(close_until_contact_srv_name, &HandCommand::callbackCloseUntilContact, this);
    tighten_grip_service = nh->advertiseService(tighten_grip_srv_name, &HandCommand::callbackTightenGrip, this);

    if (!use_sim_data_hand)
    {
        // calibrate fingers and tactile if working on real hand
        ros::ServiceClient calibrate_fingers = nh->serviceClient<std_srvs::Empty>("reflex_takktile/calibrate_fingers");
        ros::ServiceClient calibrate_tactile = nh->serviceClient<std_srvs::Empty>("reflex_takktile/calibrate_tactile");
        std_srvs::Empty srv;
        ROS_INFO("Calibrating fingers ...");
        calibrate_fingers.call(srv);
        ROS_INFO("Calibrating sensors ...");
        calibrate_tactile.call(srv);
        ROS_INFO("Calibration done!");
    }
    else
    {
        // we only offer sph_close_service and pinch_service in simulated hand
        // because fingers could crash into each other on real hand
        sph_close_service = nh->advertiseService(sph_close_srv_name, &HandCommand::callbackSphClose, this);
        pinch_service = nh->advertiseService(pinch_srv_name, &HandCommand::callbackPinch, this);
    }
}

std::string HandCommand::getStatusMsg()
{
    std::string str;
    int size = cur_cmd.size();
    for (int i = 0; i < size; i++)
    {
        str += std::to_string(cur_cmd[i]);
        if (i != size - 1)
        {
            str += ", ";
        }
    }
    std::string msg = "Sent [" + str + "] to " + pos_cmd_topic + ". ";
    return msg;
}

bool HandCommand::executePrimitive(HandCommand::Primitive primitive, bool blocking, float tolerance, float time_out, std::string *status_msg)
{
    switch (primitive)
    {
    case Open:
    {
        cur_cmd = open_pos;
        break;
    }
    case Close:
    {
        cur_cmd = close_pos;
        break;
    }
    case Pinch:
    {
        cur_cmd = pinch_pos;
        break;
    }
    case SphericalOpen:
    {
        cur_cmd = sph_open_pos;
        break;
    }
    case SphericalClose:
    {
        cur_cmd = sph_close_pos;
        break;
    }
    }
    this->publishCommand();
    *status_msg = getStatusMsg();

    if (!blocking)
    {
        return true;
    }
    if (waitUntilFinished(tolerance, time_out))
    {
        *status_msg += "All fingers reached their desired positions.";
        return true;
    }
    else
    {
        *status_msg += "Not all fingers reached their desired positions of within a time out of " +
                       std::to_string(time_out) + " secs and a tolerance of " + std::to_string(tolerance) + ".";
        return false;
    }
}

bool HandCommand::waitUntilFinished(float tolerance, float time_out)
{
    ros::Duration allowed_duration(time_out);
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);
    std::array<float, 4> diff = {0, 0, 0, 0};

    while (allowed_duration > (ros::Time::now() - start_time))
    {
        for (int i = 0; i < state->num_motors; i++)
        {
            diff[i] = abs(cur_cmd[i] - state->motor_states[i]->getJointAngle());
        }

        if (std::all_of(diff.begin(), diff.end(), [tolerance](float x) { return x < tolerance; }))
        {
            return true;
        }
        rate.sleep();
    }
    return false;
}

bool HandCommand::callbackOpen(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res)
{
    res.success = executePrimitive(Open, req.blocking, req.tolerance, req.time_out, &res.message);
    return true;
}

bool HandCommand::callbackClose(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res)
{
    res.success = executePrimitive(Close, req.blocking, req.tolerance, req.time_out, &res.message);
    return true;
}

bool HandCommand::callbackPinch(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res)
{
    res.success = executePrimitive(Pinch, req.blocking, req.tolerance, req.time_out, &res.message);
    return true;
}

bool HandCommand::callbackSphOpen(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res)
{
    res.success = executePrimitive(SphericalOpen, req.blocking, req.tolerance, req.time_out, &res.message);
    return true;
}

bool HandCommand::callbackSphClose(reflex_interface::GraspPrimitive::Request &req, reflex_interface::GraspPrimitive::Response &res)
{
    res.success = executePrimitive(SphericalClose, req.blocking, req.tolerance, req.time_out, &res.message);
    return true;
}

bool HandCommand::callbackPosIncr(reflex_interface::PosIncrement::Request &req, reflex_interface::PosIncrement::Response &res)
{
    float increment[4] = {(float)req.f1,
                          (float)req.f2,
                          (float)req.f3,
                          (float)req.preshape};
    res.success = executePosIncrement(increment, req.from_measured_pos, req.blocking, req.tolerance, req.time_out);
    res.message = this->getStatusMsg();
    if (res.success == false)
    {
        res.message += "Not all fingers reached their desired positions of within a time out of " +
                       std::to_string(req.time_out) + " secs and a tolerance of " + std::to_string(req.tolerance) + ".";
    }
    return true;
}

bool HandCommand::callbackCloseUntilContact(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::mutex mtx;

    // this service is blocking by default
    ros::Duration allowed_duration(close_until_contact_time_out);
    ros::Time start_time = ros::Time::now();
    std::vector<bool> fingers_in_contact = {0, 0, 0}; // example: finger 1 in contact {1, 0, 0}
    std::vector<bool> contact_memory = {0, 0, 0};     // example: finger 1 and 2 were in contact throughout this service call {1, 1, 0}
    std::vector<bool> contact_or_up_lim = {0, 0, 0};  // example: finger 1 reached upper joint limit and finger 2 in contact {1, 1, 0}

    ros::Rate rate(close_until_contact_pub_rate);

    while (allowed_duration > (ros::Time::now() - start_time))
    {
        // stop if all fingers have been in contact at least once throughout this service call or if all are currently in contact
        bool allFingersHaveMadeContact = std::all_of(contact_memory.begin(), contact_memory.end(), [](bool v) { return v; });
        bool stop_good = allFingersHaveMadeContact || state->allFingersInContact();
        bool stop_bad = std::all_of(contact_or_up_lim.begin(), contact_or_up_lim.end(), [](bool v) { return v; });

        if (stop_good)
        {
            res.success = true;
            res.message = "All fingers have been in contact.";
            return true;
        }
        else if (stop_bad)
        {
            res.success = false;
            res.message = "Stopping early as all fingers either reached their specified max joint value of " + std::to_string(close_until_contact_up_lim) + " or made contact.";
            return true;
        }

        float increment[4] = {0, 0, 0, 0};

        // HandState is updating vars, so locking here to avoid data race
        mtx.lock();
        fingers_in_contact = state->getVars().fingers_in_contact;
        mtx.unlock();

        for (int i = 0; i < state->num_fingers; i++)
        {
            if (!fingers_in_contact[i] && !contact_memory[i])
            {
                if (cur_cmd[i] > close_until_contact_up_lim)
                {
                    // remember that this finger already reached its upper joint limit
                    contact_or_up_lim[i] = true;
                }
                else
                {
                    // for all fingers that did not have contact yet: tighten up
                    increment[i] = close_until_contact_incr;
                }
            }
            else if (fingers_in_contact[i])
            {
                // memorize that we obtained a contact at this finger
                contact_memory[i] = true;
                contact_or_up_lim[i] = true;
            }
        }

        this->executePosIncrement(increment);

        // make sure to process callbacks in HandState class
        rate.sleep();
    }
    res.success = false;
    res.message = "Did not obtain contact on all fingers within time-out of " + std::to_string(close_until_contact_time_out) + " secs.";
    return true;
}

bool HandCommand::callbackTightenGrip(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // tighten joints by tighten_incr rad and leave preshape as is
    float increment[4] = {tighten_incr,
                          tighten_incr,
                          tighten_incr,
                          0};
    this->executePosIncrement(increment, false);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::executePosIncrement(float increment[4], bool from_measured_pos, bool blocking, float tolerance, float time_out)
{
    std::array<float, 4> cur_state;
    if (from_measured_pos)
    {
        cur_state = {state->motor_states[0]->getJointAngle(),
                     state->motor_states[1]->getJointAngle(),
                     state->motor_states[2]->getJointAngle(),
                     state->motor_states[3]->getJointAngle()};
    }
    else
    {
        cur_state = cur_cmd;
    }

    for (int i = 0; i < 4; i++)
    {
        float val = cur_state[i] + increment[i];
        if (val > low_limits[i] && val < high_limits[i])
        {
            cur_cmd[i] = val;
        }
    }
    this->publishCommand();
    if (!blocking)
    {
        return true;
    }
    return waitUntilFinished(tolerance, time_out);
}

void HandCommand::publishCommand()
{
    pos_cmd.f1 = cur_cmd[0];
    pos_cmd.f2 = cur_cmd[1];
    pos_cmd.f3 = cur_cmd[2];
    pos_cmd.preshape = cur_cmd[3];

    pub.publish(pos_cmd);
}