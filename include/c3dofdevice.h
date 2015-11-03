#ifndef C3DOFDEVICE_H
#define C3DOFDEVICE_H

#include "cMotorController.h"


class c3DOFDevice
{

public:
    // Constructor of c3DOFDevice
    c3DOFDevice();
    ~c3DOFDevice();

    // Instantiates all the motor controllers
    int Init3DOFDevice();

private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;

};

#endif // C3DOFDEVICE_H
