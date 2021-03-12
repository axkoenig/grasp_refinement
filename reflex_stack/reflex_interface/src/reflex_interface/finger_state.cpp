#include "reflex_interface/finger_state.hpp"
#include "gazebo_interface/gazebo_interface.hpp"

FingerState::FingerState(ros::NodeHandle *nh, int finger_id)
{
    this->nh = *nh;
    this->finger_id = finger_id;
    proximal_joint_frame = getProximalJointFrame();
}

tf2::Transform FingerState::getProximalJointFrame()
{
    // we obtained this data from the URDF (transforms are from shell frame to respective proximal joint frame)
    switch (finger_id)
    {
    case 1:
        return tf2::Transform(tf2::Quaternion{0, 0.139543, 0, 0.990216}, tf2::Vector3{0.0603974, -0.026, 0.0816});
    case 2:
        return tf2::Transform(tf2::Quaternion{0, 0.139543, 0, 0.990216}, tf2::Vector3{0.0603974, 0.026, 0.0816});
    case 3:
        return tf2::Transform(tf2::Quaternion{-0.139543, 0, 0.990216, 0}, tf2::Vector3{-0.03, 0, 0.0816});
    default:
        ROS_WARN("Unsupported finger id.");
        return tf2::Transform::getIdentity();
    }
}

void FingerState::setProximalAngleFromMsg(float angle)
{
    proximal_angle = angle;
}

void FingerState::setDistalAngleFromMsg(float angle)
{
    distal_angle = angle;
}

void FingerState::setPreshapeAngleFromMsg(float angle)
{
    preshape_angle = angle;
}

void FingerState::setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->sensor_contacts[i] = sensor_contacts[i];
    }
}

void FingerState::fillContactInfoWorldSim(std::vector<tf2::Vector3> &contact_positions_world, std::vector<tf2::Vector3> &contact_normals_world)
{
    // this method checks each sensor for contact and fills vectors with positions and normals
    // of the virtual sensors. we could also get the contact position and normals directly from
    // the simulation, but this won't be possible in real world. in real world we can only calculate
    // approx position and normal of each sensor in 3D space. hence this function is a "hybrid"
    // between getting accurate information from the simulation (the distal and proximal pose) and
    // also outputting values that would be obtainable in real world (the sensor pos and normal).

    if (contact_positions_world.size() != contact_normals_world.size())
    {
        ROS_ERROR("Number of contact positions and normals must be equal.");
        return;
    }

    // obtain only the link poses that are in contact
    bool has_prox_contact = hasProximalContact();
    bool has_dist_contact = hasDistalContact();
    tf2::Transform prox_link_pose;
    tf2::Transform dist_link_pose;

    if (!has_prox_contact && !has_dist_contact)
    {
        return;
    }
    if (has_prox_contact)
    {
        prox_link_pose = getLinkPoseSim(&nh, "proximal_" + std::to_string(finger_id), "world", false);
    }
    if (has_dist_contact)
    {
        dist_link_pose = getLinkPoseSim(&nh, "distal_" + std::to_string(finger_id), "world", false);
    }

    for (int i = 0; i < num_sensors; i++)
    {
        if (sensor_contacts[i])
        {
            // proximal contact
            if (i < 5)
            {
                // contact normal is z axis of link frame
                tf2::Vector3 normal = prox_link_pose * tf2::Vector3{0, 0, 1};
                normal.normalize();
                contact_normals_world.push_back(normal);

                // contact position defined in link frame (lies on surface of finger above the sensor)
                tf2::Vector3 contact_pos = {0.008 + 0.0097 * (i + 1), 0, 0.014};

                // transform contact position to world coordinates
                contact_pos = prox_link_pose * contact_pos;

                contact_positions_world.push_back(prox_link_pose.getOrigin() + contact_pos);
            }
            // distal contact
            else
            {
                tf2::Vector3 normal = dist_link_pose * tf2::Vector3{0, 0, 1};
                normal.normalize();
                contact_normals_world.push_back(normal);
                tf2::Vector3 contact_pos = {-0.004 + 0.0091 * (i - 4), 0, 0.0155};
                contact_pos = dist_link_pose * contact_pos;
                contact_positions_world.push_back(dist_link_pose.getOrigin() + contact_pos);
            }
        }
    }
}

void FingerState::setSensorPressuresFromMsg(boost::array<float, 9> sensor_pressures)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->sensor_pressures[i] = sensor_pressures[i];
    }
}

bool FingerState::hasContact()
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

bool FingerState::hasProximalContact()
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

bool FingerState::hasDistalContact()
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

void FingerState::updateFingerNormalsInShellFrame()
{
    // NOTE: this method calculates approximate surface normals in "shell" frame (not "world"!) from measured joint angles.
    // Idea: z axis of proximal_joint_frame points in normal direction of proximal pad (see RViz). We factor in the rotation
    // around preshape, proximal and distal joints and return this z axis.
    // this works on the real hand as well as in simulation.

    tf2::Transform transform = proximal_joint_frame;

    // 1) rotate around negative y axis for current measured proximal_angle
    tf2::Quaternion q;
    q.setRPY(0, -proximal_angle, 0);
    tf2::Transform rotate_prox_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    transform *= rotate_prox_angle;

    // 2) take into account preshape rotation for fingers 1 and 2
    if (finger_id == 1 || finger_id == 2)
    {
        // rotate around z axis (negative for finger 1, positive for finger 2)
        (finger_id == 1) ? q.setRPY(0, 0, -preshape_angle) : q.setRPY(0, 0, preshape_angle);
        tf2::Transform rotate_preshape_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
        transform *= rotate_preshape_angle;
    }

    prox_normal_in_shell_frame = transform * tf2::Vector3{0, 0, 1};

    // 3) rotate negative y axis for current measured distal_angle
    q.setRPY(0, -distal_angle, 0);
    tf2::Transform rotate_distal_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    transform *= rotate_distal_angle;

    dist_normal_in_shell_frame = transform * tf2::Vector3{0, 0, 1};
}

tf2::Vector3 FingerState::getProximalNormal()
{
    updateFingerNormalsInShellFrame();
    return prox_normal_in_shell_frame;
}

tf2::Vector3 FingerState::getDistalNormal()
{
    updateFingerNormalsInShellFrame();
    return dist_normal_in_shell_frame;
}