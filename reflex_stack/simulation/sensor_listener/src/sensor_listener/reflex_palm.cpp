#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "sensor_listener/reflex_palm.hpp"

ReflexPalm::ReflexPalm()
{
    std::string topic_name = "gazebo/" + sensor_link_name + "_sensor_bumper";
    sensor_link_sub = nh.subscribe(topic_name, 1, &ReflexPalm::contacts_callback, this);
}

tf2::Vector3 ReflexPalm::create_vec_from_msg(const geometry_msgs::Vector3 &msg)
{
    return tf2::Vector3{msg.x, msg.y, msg.z};
}

void ReflexPalm::contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    contact_frames.clear();
    int num_contact_states = msg.states.size();
    int states_idx = num_contact_states - 1; // take second last result

    // no contacts
    if (num_contact_states == 0)
    {
        return;
    }

    tf2::Transform world_to_pad = getLinkPoseSim(&nh, sensor_link_name, "world", false);
    tf2::Vector3 link_y = world_to_pad * tf2::Vector3(0, 1, 0);
    tf2::Transform pad_to_world = world_to_pad.inverse();

    int num_contacts = msg.states[states_idx].wrenches.size();
    for (int i = 0; i < num_contacts; i++)
    {
        sensor_listener::ContactFrame cf_msg;

        tf2::Vector3 contact_force_world = create_vec_from_msg(msg.states[states_idx].wrenches[i].force);
        cf_msg.contact_force_magnitude = contact_force_world.length();

        // stop if force is smaller than thresh (we are doing this since
        // Gazebo returns many forces which are 0 and we want to ignore them)
        if (cf_msg.contact_force_magnitude < ignore_force_thresh)
        {
            continue;
        }

        // transform contact position from world frame to pad origin
        tf2::Vector3 contact_position = create_vec_from_msg(msg.states[states_idx].contact_positions[i]);
        tf2::Vector3 contact_position_on_pad = pad_to_world * contact_position;

        // stop if contact on back of palm
        if (contact_position_on_pad[1] < 0.0)
        {
            ROS_WARN("Ignoring contact on back of palm.");
            continue;
        }

        tf2::Vector3 contact_normal = create_vec_from_msg(msg.states[states_idx].contact_normals[i]);
        tf2::Vector3 contact_torque_world = create_vec_from_msg(msg.states[states_idx].wrenches[i].torque);

        // this is our dirty fix for the Gazebo bug that flips contact normals (see https://github.com/dartsim/dart/issues/1425)
        float angle = acos(link_y.dot(contact_normal) / (contact_normal.length() * link_y.length()));
        if (abs(angle) > M_PI / 2)
        {
            contact_torque_world *= -1;
            contact_force_world *= -1;
            contact_normal *= -1;
        }

        // calculate rotation of contact frame (x must align with contact normal)
        tf2::Vector3 world_x = tf2::Vector3{1, 0, 0};
        tf2::Quaternion rot_x_to_normal = tf2::shortestArcQuatNormalize2(world_x, contact_normal);
        tf2::Transform contact_frame = tf2::Transform(rot_x_to_normal, contact_position);

        // fill remaining message
        cf_msg.sensor_id = 0;
        cf_msg.finger_id = 0;
        cf_msg.palm_contact = true;
        cf_msg.contact_torque_magnitude = contact_torque_world.length();
        cf_msg.contact_wrench.force = tf2::toMsg(contact_force_world);
        cf_msg.contact_wrench.torque = tf2::toMsg(contact_torque_world);
        cf_msg.contact_frame = tf2::toMsg(contact_frame);
        cf_msg.contact_position = tf2::toMsg(contact_position);
        cf_msg.contact_normal = tf2::toMsg(contact_normal);

        // ROS_INFO_STREAM("normal [" << i << "]" << contact_normal[0] << ", " << contact_normal[1] << ", " << contact_normal[2]);
        // ROS_INFO_STREAM("force [" << i << "]" << contact_force_world[0] << ", " << contact_force_world[1] << ", " << contact_force_world[2]);
        // ROS_INFO_STREAM("angle " << acos(contact_force_world.dot(contact_normal) / (contact_normal.length() * contact_force_world.length())) * 180 / M_PI);

        // all is well, add contact frame to vector
        contact_frames.push_back(cf_msg);
    }
}
