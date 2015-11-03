#include "c3dofdevice.h"


c3DOFDevice::c3DOFDevice()
{

}

c3DOFDevice::~c3DOFDevice()
{

}

int c3DOFDevice::Init3DOFDevice()
{
    motor_1 = new cMotorController(1);
    motor_1->InitEncoder();
    motor_2 = new cMotorController(2);
    motor_3 = new cMotorController(3);

    return 0;
}

