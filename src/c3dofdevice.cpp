#include "c3dofdevice.h"


c3DOFDevice::c3DOFDevice()
{

}

c3DOFDevice::~c3DOFDevice()
{

}

int c3DOFDevice::Init3DOFDeviceEnc()
{
    motor_1 = new cMotorController(1);
    motor_1->InitEncoder();
    motor_2 = new cMotorController(2);
    motor_2->InitEncoder();
    motor_3 = new cMotorController(3);
    motor_3->InitEncoder();

    return 0;
}

QVector<double> c3DOFDevice::GetMotorAngles()
{
    QVector<double> returnAngles(3);
    returnAngles[0] = motor_1->GetMotorAngle();
    returnAngles[1] = motor_2->GetMotorAngle();
    returnAngles[2] = motor_3->GetMotorAngle();
    return returnAngles;
}



QVector<double> c3DOFDevice::MotAngToJointAng(QVector<double> JointAngles)
{
    QVector<double> returnVal(3);
    return returnVal;
}

