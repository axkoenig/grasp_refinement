#include <math.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "sensor_listener/reflex_palm.hpp"

ReflexPalm::ReflexPalm()
{
    std::string topic_name = "gazebo/" + sensor_link_name + "_sensor_bumper";
    sensor_link_sub = nh.subscribe(topic_name, 1, &ReflexPalm::contacts_callback, this);
}

void ReflexPalm::contacts_callback(const gazebo_msgs::ContactsState &msg)
{

}
