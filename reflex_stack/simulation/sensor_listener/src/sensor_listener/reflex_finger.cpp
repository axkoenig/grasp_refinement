#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "sensor_listener/reflex_finger.hpp"

ReflexFinger::ReflexFinger(const int &finger_id)
{
    this->finger_id = finger_id;
    std::string finger_id_str = std::to_string(this->finger_id);

    std::string proximal_topic = "gazebo/finger_" + finger_id_str + "_proximal_position_controller/state";
    std::string proximal_to_flex_topic = "gazebo/finger_" + finger_id_str + "_proximal_to_flex_position_controller/state";
    std::string flex_to_distal_topic = "gazebo/finger_" + finger_id_str + "_flex_to_distal_position_controller/state";
    proximal_sensor_link_name = "proximal_" + finger_id_str;
    distal_sensor_link_name = "distal_" + finger_id_str;
    std::string proximal_sensor_link_topic = "gazebo/" + proximal_sensor_link_name + "_pad_sensor_bumper";
    std::string distal_sensor_link_topic = "gazebo/" + distal_sensor_link_name + "_pad_sensor_bumper";

    proximal_sub = nh.subscribe(proximal_topic, 1, &ReflexFinger::proximal_callback, this);
    proximal_to_flex_sub = nh.subscribe(proximal_to_flex_topic, 1, &ReflexFinger::proximal_to_flex_callback, this);
    flex_to_distal_sub = nh.subscribe(flex_to_distal_topic, 1, &ReflexFinger::flex_to_distal_callback, this);
    proximal_sensor_link_sub = nh.subscribe(proximal_sensor_link_topic, 1, &ReflexFinger::proximal_contacts_callback, this);
    distal_sensor_link_sub = nh.subscribe(distal_sensor_link_topic, 1, &ReflexFinger::distal_contacts_callback, this);

    // compute inner boundaries between sensors along x axis
    float prox_sensor_width = len_prox_pad / 5;
    for (int i = 0; i < 4; i++)
    {
        prox_sensor_boundaries[i] = -len_prox_pad / 2 + prox_sensor_width * (i + 1);
    }
    float dist_sensor_width = len_dist_pad / 4;
    for (int i = 0; i < 3; i++)
    {
        dist_sensor_boundaries[i] = -len_dist_pad / 2 + dist_sensor_width * (i + 1);
    }
}

void ReflexFinger::proximal_callback(const control_msgs::JointControllerState &msg)
{
    proximal_angle = msg.process_value;
}

void ReflexFinger::proximal_to_flex_callback(const control_msgs::JointControllerState &msg)
{
    proximal_to_flex_angle = msg.process_value;
}

void ReflexFinger::flex_to_distal_callback(const control_msgs::JointControllerState &msg)
{
    flex_to_distal_angle = msg.process_value;
}

float ReflexFinger::getProximalAngle()
{
    return proximal_angle;
}

float ReflexFinger::getDistalAngle()
{
    // note that (as with the real reflex hand) this is a rough approximation
    // we take the average of both flexure joints
    // TODO: compare with real reflex sensor readings
    return (proximal_to_flex_angle + flex_to_distal_angle) / 2;
}
void ReflexFinger::proximal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    // pad origin is center of pad collision box which surrounds proximal link
    tf2::Transform world_to_prox_link = getLinkPoseSim(&nh, proximal_sensor_link_name, "world", false);
    eval_contacts_callback(msg, prox_contact_frames, 0, num_prox_sensors, prox_sensor_boundaries, world_to_prox_link, prox_link_to_prox_pad_origin);
}

void ReflexFinger::distal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    tf2::Transform world_to_dist_link = getLinkPoseSim(&nh, distal_sensor_link_name, "world", false);
    eval_contacts_callback(msg, dist_contact_frames, 5, num_dist_sensors, dist_sensor_boundaries, world_to_dist_link, dist_link_to_dist_pad_origin);
}

tf2::Vector3 ReflexFinger::create_vec_from_msg(const geometry_msgs::Vector3 &msg)
{
    return tf2::Vector3{msg.x, msg.y, msg.z};
}

void ReflexFinger::eval_contacts_callback(const gazebo_msgs::ContactsState &msg,
                                          std::vector<sensor_listener::ContactFrame> &contact_frames,
                                          const int &first_sensor_idx,
                                          const int &num_sensors_on_link,
                                          const float sensor_boundaries[],
                                          const tf2::Transform &world_to_link,
                                          const tf2::Vector3 &link_to_pad_origin)
{
    // reset variable
    contact_frames.clear();

    // number of intermediate collision results (usually around 20)
    int num_contact_states = msg.states.size();

    // we only take the second latest result to decrease compuational load. (latest result is
    // sometimes not filled by Gazebo and results in error)
    int states_idx = num_contact_states - 1;

    // no contacts on link, set all sensors zero
    if (num_contact_states == 0)
    {
        for (int i = 0; i < num_sensors_on_link; i++)
        {
            sensors[first_sensor_idx + i].addContactToBuffer(false);
            sensors[first_sensor_idx + i].addPressureToBuffer(0.0);
        }
        return;
    }

    // define bins for each sensor and keep track of number of contacts on each sensor
    float pressures[num_sensors_on_link] = {0};
    bool contacts[num_sensors_on_link] = {0};
    int num_real_contacts[num_sensors_on_link] = {0};

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

        // transform contact_pos from world frame to pad origin
        tf2::Vector3 contact_pos = create_vec_from_msg(msg.states[states_idx].contact_positions[i]);
        contact_pos = world_to_link.inverse() * contact_pos - link_to_pad_origin;

        // stop if contact on back of finger
        if (contact_pos[2] < 0.0)
        {
            ROS_WARN("Ignoring contact on back of finger.");
            continue;
        }

        // find out which sensor this contact would belong to and save results
        int sensor_id = which_sensor(contact_pos[0], sensor_boundaries, num_sensors_on_link - 1);
        pressures[sensor_id] += cf_msg.contact_force_magnitude;
        contacts[sensor_id] = true;
        num_real_contacts[sensor_id] += 1;

        tf2::Vector3 contact_normal = create_vec_from_msg(msg.states[states_idx].contact_normals[i]);
        tf2::Vector3 contact_torque_world = create_vec_from_msg(msg.states[states_idx].wrenches[i].torque);

        // calculate rotation of contact frame (z must align with contact normal)
        tf2::Vector3 world_z = tf2::Vector3{0, 0, 1};
        tf2::Quaternion rot_z_to_normal = tf2::shortestArcQuatNormalize2(world_z, contact_normal);
        tf2::Transform contact_frame = tf2::Transform(rot_z_to_normal, contact_pos);

        // fill remaining message
        cf_msg.sensor_id = first_sensor_idx + sensor_id + 1; // ranges from 1 to 9
        cf_msg.finger_id = finger_id;                        // ranges from 1 to 3
        cf_msg.contact_torque_magnitude = contact_torque_world.length();
        cf_msg.contact_wrench.force = tf2::toMsg(contact_force_world);
        cf_msg.contact_wrench.torque = tf2::toMsg(contact_torque_world);
        cf_msg.contact_frame = tf2::toMsg(contact_frame);
        cf_msg.contact_pos = tf2::toMsg(contact_pos);
        cf_msg.contact_normal = tf2::toMsg(contact_normal);

        // all is well, add contact frame to vector
        contact_frames.push_back(cf_msg);

        // DEBUG
        // tf2::Vector3 z = contact_frame * tf2::Vector3{0, 0, 1};
        // ROS_INFO_STREAM("contact frame z is " << z[0] << "," << z[1] << "," << z[2]);
        // ROS_INFO_STREAM("measured contact normal is " << contact_normal[0] << "," << contact_normal[1] << "," << contact_normal[2]);
        // ROS_INFO_STREAM("angle" << tf2::tf2Angle(contact_normal, z));
    }

    for (int i = 0; i < num_sensors_on_link; i++)
    {
        // average forces if we have multiple contacts on one sensor
        if (num_real_contacts[i] > 1)
        {
            pressures[i] /= num_real_contacts[i];
        }
        // save final values for each sensor
        sensors[first_sensor_idx + i].addContactToBuffer(contacts[i]);
        sensors[first_sensor_idx + i].addPressureToBuffer(pressures[i]);
    }
};

int ReflexFinger::which_sensor(const float &contact_x, const float sensor_boundaries[], const int &num_boundaries)
{
    for (int i = 0; i < num_boundaries; i++)
    {
        if (contact_x < sensor_boundaries[i])
        {
            return i;
        }
    }
    return num_boundaries;
}