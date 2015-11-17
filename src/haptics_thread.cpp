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
        p_CommonData->wearableDelta->SetMotorTorqueOutput(p_CommonData->wearableDelta->ReadDesiredForce());
    }
}

void haptics_thread::initialize()
{

}

void haptics_thread::PositionController(Eigen::Vector3d desiredPositionArg)
{

}
