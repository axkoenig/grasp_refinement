#ifndef REFLEX_HAND_H
#define REFLEX_HAND_H

#include "sensor_listener/reflex_finger.hpp"
#include "sensor_listener/reflex_motor.hpp"

class ReflexHand
{
public:
    int num_fingers = 3;
    int num_sensors = 9;
    int num_motors = 4;

    ReflexFinger fingers[3] = {ReflexFinger(1), ReflexFinger(2), ReflexFinger(3)};
    ReflexMotor *motors[4];

    ReflexHand()
    {
        motors[0] = new ReflexFingerMotor(1);
        motors[1] = new ReflexFingerMotor(2);
        motors[2] = new ReflexFingerMotor(3);
        motors[3] = new ReflexPreshapeMotor();
    }
};

#endif