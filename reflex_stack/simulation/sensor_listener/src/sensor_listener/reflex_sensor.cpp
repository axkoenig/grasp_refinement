#include <numeric>

#include "sensor_listener/reflex_sensor.hpp"
#include "gazebo_interface/gazebo_interface.hpp"

ReflexSensor::ReflexSensor(ros::NodeHandle &nh)
{
    getParam(&nh, &default_contact_threshold, "default_contact_threshold");
}

int ReflexSensor::getPressure()
{
    // we average pressure over buffer
    float avg_pressure = std::accumulate(pressure_buffer.begin(), pressure_buffer.end(), 0.0) / buf_size;

    // linearly scale to range [0, max_pressure_val]
    int res = int((avg_pressure / pressure_at_max_val) * max_pressure_val);
    return res <= max_pressure_val ? res : max_pressure_val;
}

bool ReflexSensor::getContact()
{
    return getPressure() > default_contact_threshold;
}

void ReflexSensor::addContactToBuffer(const bool contact)
{
    std::rotate(contact_buffer.rbegin(), contact_buffer.rbegin() + 1, contact_buffer.rend());
    contact_buffer[0] = contact;
}

void ReflexSensor::addPressureToBuffer(const float pressure)
{
    std::rotate(pressure_buffer.rbegin(), pressure_buffer.rbegin() + 1, pressure_buffer.rend());
    pressure_buffer[0] = pressure;
}