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
        double motorAngle = p_CommonData->testController->GetMotorAngle();
    }
}

void haptics_thread::initialize()
{

}
