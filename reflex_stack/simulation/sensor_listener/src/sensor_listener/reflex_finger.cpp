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

float ReflexFinger::calc_l2_norm(const float (&vector)[3])
{
    return sqrt(pow(vector[0], 2) + pow(vector[1], 2) + pow(vector[2], 2));
}

void ReflexFinger::proximal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    // pad origin is center of pad collision box which surrounds proximal link
    tf2::Transform world_to_prox_link = getLinkPoseSim(&nh, proximal_sensor_link_name, "world", false);
    eval_contacts_callback(msg, 0, num_prox_sensors, prox_sensor_boundaries, world_to_prox_link, prox_link_to_prox_pad_origin);
}

void ReflexFinger::distal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    tf2::Transform world_to_dist_link = getLinkPoseSim(&nh, distal_sensor_link_name, "world", false);
    eval_contacts_callback(msg, 5, num_dist_sensors, dist_sensor_boundaries, world_to_dist_link, dist_link_to_dist_pad_origin);
}

void ReflexFinger::eval_contacts_callback(const gazebo_msgs::ContactsState &msg,
                                          const int &first_sensor_idx,
                                          const int &num_sensors_on_link,
                                          const float sensor_boundaries[],
                                          const tf2::Transform &world_to_link,
                                          const tf2::Vector3 &link_to_pad_origin)
{
    // number of intermediate collision results (usually around 20)
    // we only take the latest result to decrease compuational load
    int num_contact_states = msg.states.size();

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

    // define bins for each sensor and keep track of number of contacts for each sensor
    float pressures[num_sensors_on_link] = {0};
    bool contacts[num_sensors_on_link] = {0};
    int num_real_contacts[num_sensors_on_link] = {0};

    int num_contacts = msg.states[num_contact_states - 1].wrenches.size();
    for (int i = 0; i < num_contacts; i++)
    {
        // obtain magnitude of contact force
        float f[3] = {0, 0, 0};
        f[0] = msg.states[num_contact_states - 1].wrenches[i].force.x;
        f[1] = msg.states[num_contact_states - 1].wrenches[i].force.y;
        f[2] = msg.states[num_contact_states - 1].wrenches[i].force.z;
        float f_norm = calc_l2_norm(f);

        // stop if force is smaller than thresh (we are doing this since
        // Gazebo returns many forces which are 0 and we want to ignore them)
        if (f_norm < ignore_force_thresh)
        {
            continue;
        }

        tf2::Vector3 contact_pos = {msg.states[num_contact_states - 1].contact_positions[i].x,
                                    msg.states[num_contact_states - 1].contact_positions[i].y,
                                    msg.states[num_contact_states - 1].contact_positions[i].z};

        // transform contact_pos from world frame to pad origin
        tf2::Transform link_to_world = world_to_link.inverse();
        contact_pos = link_to_world * contact_pos - link_to_pad_origin;
        tf2::Vector3 contact_in_link_frame = link_to_world * contact_pos;

        // stop if contact on back of finger
        if (contact_pos[2] < 0.0)
        {
            ROS_WARN("Ignoring contact on back of finger.");
            continue;
        }
        int sensor_id = which_sensor(contact_pos[0], sensor_boundaries, num_sensors_on_link - 1);
        pressures[sensor_id] += f_norm;
        contacts[sensor_id] = true;
        num_real_contacts[sensor_id] += 1;
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