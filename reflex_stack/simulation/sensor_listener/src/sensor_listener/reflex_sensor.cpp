#include <numeric>

#include "sensor_listener/reflex_sensor.hpp"

double ReflexSensor::getPressure()
{
    return pressure;
}

bool ReflexSensor::getContact()
{
    return contact;
}

void ReflexSensor::setTopic(std::string topic)
{
    sensor_sub = nh.subscribe(topic, 1, &ReflexSensor::callback, this);
}

void ReflexSensor::callback(const gazebo_msgs::ContactsState &msg)
{
    std::string sensor_name = msg.header.frame_id;
    num_contacts = msg.states.size();

    // reset variables
    pressure = 0.0;
    contact = false;

    // rotate buffers right
    std::rotate(contact_buffer.rbegin(), contact_buffer.rbegin() + 1, contact_buffer.rend());
    std::rotate(pressure_buffer.rbegin(), pressure_buffer.rbegin() + 1, pressure_buffer.rend());

    if (num_contacts > 0)
    {
        double f[3] = {0, 0, 0};

        // get sum of all contact forces on sensor
        for (int i = 0; i < num_contacts; i++)
        {
            f[0] += msg.states[i].total_wrench.force.x;
            f[1] += msg.states[i].total_wrench.force.y;
            f[2] += msg.states[i].total_wrench.force.z;
        }

        // average over all contacts
        f[0] /= num_contacts;
        f[1] /= num_contacts;
        f[2] /= num_contacts;

        // pressure is magnitude of total force vector
        pressure_buffer[0] = sqrt(pow(f[0], 2) + pow(f[1], 2) + pow(f[2], 2)) * scaling_factor;
        contact_buffer[0] = true;
    }
    else
    {
        pressure_buffer[0] = 0.0;
        contact_buffer[0] = false;
    }

    // if any element in contact buffer is true, we return true
    contact = true ? std::any_of(contact_buffer.begin(), contact_buffer.end(), [](bool v) { return v; }) : false;

    // we average pressure over buffer
    pressure = std::accumulate(pressure_buffer.begin(), pressure_buffer.end(), 0.0) / buf_size;
}