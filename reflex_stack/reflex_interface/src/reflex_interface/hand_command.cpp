#include "reflex_interface/PosIncrement.h"
#include "reflex_interface/hand_command.hpp"

HandCommand::HandCommand(ros::NodeHandle *nh, HandState *state)
{
    this->state = state;

    pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
    open_service = nh->advertiseService(open_srv_name, &HandCommand::callbackOpen, this);
    close_service = nh->advertiseService(close_srv_name, &HandCommand::callbackClose, this);
    pinch_service = nh->advertiseService(pinch_srv_name, &HandCommand::callbackPinch, this);
    sph_open_service = nh->advertiseService(sph_open_srv_name, &HandCommand::callbackSphOpen, this);
    sph_close_service = nh->advertiseService(sph_close_srv_name, &HandCommand::callbackSphClose, this);
    pos_incr_service = nh->advertiseService(pos_incr_srv_name, &HandCommand::callbackPosIncr, this);
    close_until_contact_service = nh->advertiseService(close_until_contact_srv_name, &HandCommand::callbackCloseUntilContact, this);
    tighten_grip_service = nh->advertiseService(tighten_grip_srv_name, &HandCommand::callbackTightenGrip, this);
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
    std::string msg = "Sent [" + str + "] to " + pos_cmd_topic;
    return msg;
}

void HandCommand::executePrimitive(HandCommand::Primitive primitive, bool verbose)
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
    this->sendCommands();
    if (verbose)
    {
        ROS_INFO_STREAM(getStatusMsg());
    }
}

bool HandCommand::callbackOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Open);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Close);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackPinch(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Pinch);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackSphOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(SphericalOpen);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackSphClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(SphericalClose);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackPosIncr(reflex_interface::PosIncrement::Request &req, reflex_interface::PosIncrement::Response &res)
{
    float increment[4] = {(float)req.f1,
                          (float)req.f2,
                          (float)req.f3,
                          (float)req.preshape};
    this->executePosIncrement(increment);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackCloseUntilContact(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ros::Duration allowed_duration(close_until_contact_time_out);
    ros::Time start_time = ros::Time::now();
    std::vector<bool> fingers_in_contact = {0, 0, 0};   // example: finger 1 in contact {1, 0, 0}
    std::vector<bool> contact_memory = {0, 0, 0};       // example: finger 1 and 2 were in contact throughout this service call {1, 1, 0}

    ros::Rate rate(close_until_contact_pub_rate);

    while (allowed_duration > (ros::Time::now() - start_time))
    {
        // stop if all fingers have been in contact at least once throughout this service call or if all are currently in contact
        bool allFingersHaveMadeContact = std::all_of(contact_memory.begin(), contact_memory.end(), [](bool v) { return v; });
        bool stop = allFingersHaveMadeContact || state->allFingersInContact();

        if (stop)
        {
            res.success = true;
            res.message = "All fingers have been in contact.";
            return true;
        }

        fingers_in_contact = state->getFingersInContact();
        float increment[4] = {0, 0, 0, 0};

        for (int i = 0; i < state->num_fingers; i++)
        {
            if (!fingers_in_contact[i] && !contact_memory[i])
            {
                // for all fingers that did not have contact yet: tighten up
                increment[i] = close_until_contact_incr;
            }
            else if (fingers_in_contact[i])
            {
                // memorize that we obtained a contact at this finger
                contact_memory[i] = 1;
            }
        }
        this->executePosIncrement(increment);

        // make sure to process callbacks in HandState class
        ros::spinOnce();
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
    this->executePosIncrement(increment);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

void HandCommand::executePosIncrement(float increment[4])
{
    float cur_state[4] = {state->finger_states[0]->getProximalAngle(),
                          state->finger_states[1]->getProximalAngle(),
                          state->finger_states[2]->getProximalAngle(),
                          state->finger_states[0]->getPreshapeAngle() + state->finger_states[1]->getPreshapeAngle()};

    for (int i = 0; i < 4; i++)
    {
        float val = cur_state[i] + increment[i];
        if (val > low_limits[i] && val < high_limits[i])
        {
            cur_cmd[i] = val;
        }
    }
    this->sendCommands();
}

void HandCommand::sendCommands()
{
    pos_cmd.f1 = cur_cmd[0];
    pos_cmd.f2 = cur_cmd[1];
    pos_cmd.f3 = cur_cmd[2];
    pos_cmd.preshape = cur_cmd[3];

    pub.publish(pos_cmd);
}