#include <boost/thread/lock_guard.hpp>

#include "reflex_interface/motor_state.hpp"

void MotorState::setJointAngle(float angle)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->joint_angle = angle;
}
void MotorState::setRawAngle(float raw_angle)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->raw_angle = raw_angle;
}
void MotorState::setVelocity(float velocity)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->velocity = velocity;
}
void MotorState::setLoad(float load)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->load = load;
}
void MotorState::setVoltage(float voltage)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->voltage = voltage;
}
void MotorState::setTemperature(float temperature)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->temperature = temperature;
}
void MotorState::setErrorState(std::string error_state)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    this->error_state = error_state;
}

int MotorState::getMotorId()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return motor_id;
}
float MotorState::getJointAngle()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return joint_angle;
}
float MotorState::getRawAngle()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return raw_angle;
}
float MotorState::getVelocity()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return velocity;
}
float MotorState::getLoad()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return load;
}
float MotorState::getVoltage()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return voltage;
}
float MotorState::getTemperature()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return temperature;
}
std::string MotorState::getErrorState()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return error_state;
}