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
#include <Eigen/dense>

class cMotorController
{
public:
    // Constructor of cMotorControl
    cMotorController(int);

    // Open and Access Sensoray 626, configure for encoder
    int open();
    int close();
    int MotorNumToChannelNum(int);
    void InitEncoder();
    void SetOffsetAngle();
    double GetMotorAngle();
    void SetOutputTorque(double);

    // Destructor of cMotorControl
    virtual ~cMotorController();

    // used to look at voltage output
    double voltageOutput;



private:
    QMutex m_mutex;
    int motorNum;
    int channelNum;
    double offsetAngle;

};



































#endif // CMOTORCONTROL_H
