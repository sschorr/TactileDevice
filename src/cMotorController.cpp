//===========================================================================
/*
    This file controls defines a class "MotorControl" that interfaces
    with the Sensoray 626 to allow control of a motor with encoder.
*/
//===========================================================================


#include "cMotorController.h"


// Constructor of motor controller =========================================
cMotorController::cMotorController()
{

}

// Destructor of motor controller ================================
cMotorController::~cMotorController()
{

}

int cMotorController::open()
{
    // load the .dll
    S626_DLLOpen();

    // open the 626 card
    S626_OpenBoard(0,0,0,0);

    if (S626_GetErrors(0) != 0)
        return S626_GetErrors(0);


    return 0;
}

int cMotorController::close()
{
    // load the .dll
    S626_DLLClose();
    return 0;
}
