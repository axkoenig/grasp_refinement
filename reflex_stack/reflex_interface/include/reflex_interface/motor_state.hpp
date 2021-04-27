#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <string>

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

public:
    MotorState(int motor_id) { this->motor_id = motor_id; };

    void setJointAngle(float angle) { this->joint_angle = angle; };
    void setRawAngle(float raw_angle) { this->raw_angle = raw_angle; };
    void setVelocity(float velocity) { this->velocity = velocity; };
    void setLoad(float load) { this->load = load; };
    void setVoltage(float voltage) { this->voltage = voltage; };
    void setTemperature(float temperature) { this->temperature = temperature; };
    void setErrorState(std::string error_state) { this->error_state = error_state; };

    int getMotorId() const { return motor_id; };
    float getJointAngle() const { return joint_angle; };
    float getRawAngle() const { return raw_angle; };
    float getVelocity() const { return velocity; };
    float getLoad() const { return load; };
    float getVoltage() const { return voltage; };
    float getTemperature() const { return temperature; };
    std::string getErrorState() const { return error_state; };
};

#endif