#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <string>
#include <boost/thread/thread.hpp>

class MotorState
{
private:
    int motor_id = 0;
    float joint_angle = 0;
    float raw_angle = 0;
    float velocity = 0;
    float load = 0;
    float voltage = 0;
    float temperature = 0;
    std::string error_state = "";
    boost::mutex mtx;

public:
    MotorState(int motor_id) { this->motor_id = motor_id; };

    void setJointAngle(float angle);
    void setRawAngle(float raw_angle);
    void setVelocity(float velocity);
    void setLoad(float load);
    void setVoltage(float voltage);
    void setTemperature(float temperature);
    void setErrorState(std::string error_state);

    int getMotorId();
    float getJointAngle();
    float getRawAngle();
    float getVelocity();
    float getLoad();
    float getVoltage();
    float getTemperature();
    std::string getErrorState();
};

#endif