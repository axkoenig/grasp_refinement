#ifndef REFLEX_SENSOR_H
#define REFLEX_SENSOR_H

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>

class ReflexSensor
{
private:
    double pressure = 0.0;
    bool contact = false;

    // use rolling buffer to filter noisy sensing from simulation
    int buf_size = 5;
    std::vector<bool> contact_buffer = {0, 0, 0, 0, 0};
    std::vector<float> pressure_buffer = {0, 0, 0, 0, 0};

    // TODO: find real scaling factor (for now pressure magnitudes don't matter)
    double scaling_factor = 1.0;

public:
    double getPressure();
    bool getContact();
    void addContactToBuffer(const bool contact);
    void addPressureToBuffer(const float pressure);
};

#endif