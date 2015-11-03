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
    cMotorController(int);

    // Open and Access Sensoray 626, configure for encoder
    int open();
    int close();
    int MotorNumToCounterNum();
    int InitEncoder();
    int GetMotorAngle();

    // Public variables
    float motorAngle;

    // Destructor of cMotorControl
    virtual ~cMotorController();
private:
    int motorNum;
    int counterNum;

};



































#endif // CMOTORCONTROL_H
