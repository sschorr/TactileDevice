#ifndef C3DOFDEVICE_H
#define C3DOFDEVICE_H

#include "cMotorController.h"
#include <QVector>

// Physical parameters of build =======================================

// radius of motor pulley
#define MOTRAD 2; // [mm]
// horizontal offset of motorshaft from base joint
#define HORIZOFFSET 6; // [mm]
// vertical offset of motorshaft from base joint
#define VERTOFFSET -9.5; // [mm]


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
    QVector<double> MotAngToJointAng(QVector<double> JointAngles);

    // zeros all of the encoders on each cMotorController =================
    void ZeroEncoders();



private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;





};

#endif // C3DOFDEVICE_H
