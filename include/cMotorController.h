//===========================================================================
/*
    This file controls defines a class "MotorControl" that interfaces
    with the Sensoray 626 to allow control of a motor with encoder.
*/
//===========================================================================

#ifndef CMOTORCONTROLLER_H
#define CMOTORCONTROLLER_H

// List all includes ==========================================
#include "Win626.h"

class cMotorController
{
public:
    // Constructor of cMotorControl
    cMotorController(unsigned int motorNumber);

    // Open and Access Sensoray 626
    int open();

    int close();

    // Destructor of cMotorControl
    virtual ~cMotorController();
private:
    int m_motorID;
};



































#endif // CMOTORCONTROL_H
