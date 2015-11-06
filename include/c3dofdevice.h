#ifndef C3DOFDEVICE_H
#define C3DOFDEVICE_H

#include "cMotorController.h"
#include <QDebug>
#include <QVector>
#include <math.h>

#define PI 3.14159265
// Physical parameters of build =======================================

// radius of motor pulley
#define MOTRAD 2 // [mm]
// horizontal offset of motorshaft from base joint
#define HORIZOFFSET 6.25 // [mm]
// vertical offset of motorshaft from base joint
#define VERTOFFSET -9.5 // [mm]
// distance from base joint to tether attachment point
#define ATTACHL 7.5 // [mm]
// Base joint angles during calibration procedure
#define CALIBANGLE 0.785398 //45 [deg] to rad



class c3DOFDevice
{

public:
    // Constructor of c3DOFDevice =========================================
    c3DOFDevice();
    ~c3DOFDevice();

    // Instantiates all the motor controllers =============================
    int Init3DOFDeviceEnc();

    // returns the vector of all motor angles =============================
    QVector<double> GetMotorAngles();

    // Determines the base joint angles based on the motor angles =========
    QVector<double> c3DOFDevice::GetJointAngles();

    // zeros all of the encoders on each cMotorController =================
    void ZeroEncoders();



private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;





};

#endif // C3DOFDEVICE_H
