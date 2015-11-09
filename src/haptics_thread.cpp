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
        Eigen::Vector3d desiredForce(p_CommonData->GUI_desiredX, p_CommonData->GUI_desiredY, p_CommonData->GUI_desiredZ);
        p_CommonData->wearableDelta->SetDesiredForce(desiredForce);
    }
}

void haptics_thread::initialize()
{

}
