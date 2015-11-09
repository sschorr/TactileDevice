#ifndef C3DOFDEVICE_H
#define C3DOFDEVICE_H

#include "cMotorController.h"
#include <QDebug>
#include <QVector>
#include <math.h>
#include <QGenericMatrix>
#include <QMatrix>
#include <Eigen/dense>
#include <iostream>

#include <QMutex>

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
// length upper arm
#define L_UA 9
// length lower arm
#define L_LA 9
// length of base (center to joint)
#define L_BASE 15
// length of end effector (center to joint)
#define L_EE 15
// Spring torque of spring at parallel
#define SPRING_TORQUE .050



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
    QVector<double> GetJointAngles();

    // zeros all of the encoders on each cMotorController =================
    void ZeroEncoders();

    // gets the end effector position =====================================
    QVector<double> GetCartesianPos();

    // determine the required torques for a given force (positive torque is pulling thread)
    QVector<double> GetDesiredTorques(Eigen::Vector3d);

    // Set the desired force
    void SetDesiredForce(Eigen::Vector3d);

    // read the desired force
    Eigen::Vector3d ReadDesiredForce();



private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;
    Eigen::Vector3d desiredForce;







};

#endif // C3DOFDEVICE_H
