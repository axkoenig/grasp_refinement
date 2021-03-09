#include <numeric>

#include "sensor_listener/reflex_sensor.hpp"

double ReflexSensor::getPressure()
{
    // we average pressure over buffer
    return scaling_factor * std::accumulate(pressure_buffer.begin(), pressure_buffer.end(), 0.0) / buf_size;
}

bool ReflexSensor::getContact()
{
    // if any element in contact buffer is true, we return true
    return true ? std::any_of(contact_buffer.begin(), contact_buffer.end(), [](bool v) { return v; }) : false;
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