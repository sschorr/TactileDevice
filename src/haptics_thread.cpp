#include "haptics_thread.h"

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::run()
{
    forever
    {
        Eigen::Vector3d desiredJointTorque = p_CommonData->wearableDelta->CalcDesiredJointTorques(p_CommonData->wearableDelta->ReadDesiredForce());
        Eigen::Vector3d desiredMotorTorque = p_CommonData->wearableDelta->CalcDesiredMotorTorques(desiredJointTorque);
        p_CommonData->wearableDelta->SetMotorTorqueOutput(desiredMotorTorque);

    }
}

void haptics_thread::initialize()
{

}
