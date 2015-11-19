#include "haptics_thread.h"

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::run()
{
    while(p_CommonData->hapticsThreadActive)
    {
        if(p_CommonData->forceControlMode == true)
        {
            p_CommonData->wearableDelta->ForceController();
        }

        else if(p_CommonData->posControlMode == true)
        {
            p_CommonData->wearableDelta->PositionController();
        }
    }

    // If we are terminating, delete the haptic device to set outputs to 0
    delete p_CommonData->wearableDelta;
}

void haptics_thread::initialize()
{    
    // Ensure the device is not controlling to start
    p_CommonData->forceControlMode = false;
    p_CommonData->posControlMode = false;

    p_CommonData->wearableDelta->SetDesiredForce(Eigen::Vector3d(0,0,0));
    p_CommonData->wearableDelta->SetDesiredPos(Eigen::Vector3d(0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180)));

    // set flag that says haptics thread is running
    p_CommonData->hapticsThreadActive = true;


}

