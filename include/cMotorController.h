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

#include <QMutex>

class cMotorController
{
public:
    // Constructor of cMotorControl
    cMotorController(int);

    // Open and Access Sensoray 626, configure for encoder
    int open();
    int close();
    int MotorNumToCounterNum(int);
    int InitEncoder();
    double GetMotorAngle();

    // Destructor of cMotorControl
    virtual ~cMotorController();
private:
    QMutex m_mutex;
    int motorNum;
    int counterNum;

};



































#endif // CMOTORCONTROL_H
