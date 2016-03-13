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
#include "windows.h"

// DEFINES =================================================================
// Define used for max Sensoray Encoder Count
#define MAX_COUNT 16777215

// Define used for DAC Voltage Conversion
#define DAC_VSCALAR 819.1

// Define the encoder counts per revolution
#define ENCODER_CPR 360 // for pololu motor (12*30:1)
//#define ENCODER_CPR 3200 // for micromo 50*64:1
//#define ENCODER_CPR 12800 // for micromo 50*256:1

// Define used for Encoder count to angle in radian
#define ENCCOUNT_TO_RAD 2*3.1415926535897932384/(ENCODER_CPR)

// Define the torque constant torque = I*KT
#define KT 78.44 //mNm/A for pololu
//#define KT 125.44 //mNm/A for micromo 64:1
//#define KT 501.76 //mNm/A for micromo 256:1

// Define the amplifier output to voltage input ratio I = I_RATIO*V
#define AMPS_TO_VOLTS 10 //Amps*10 = # Volts, Amps = Volts/10

// Define the max amperage of the motor (for all 3 combined)
#define MAX_AMPS 0.092

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
