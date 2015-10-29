//===========================================================================
/*
    This file controls defines a class "MotorControl" that interfaces
    with the Sensoray 626 to allow control of a motor with encoder.
*/
//===========================================================================


#include "cMotorController.h"


// Constructor of motor controller =========================================
cMotorController::cMotorController(unsigned int motorNumber)
{
    m_motorID = motorNumber;
}

// Destructor of motor controller ================================
cMotorController::~cMotorController()
{

}

int cMotorController::open()
{
    // load the .dll
    S626_DLLOpen();
    return 0;
}

int cMotorController::close()
{
    // load the .dll
    S626_DLLClose();
    return 0;
}
