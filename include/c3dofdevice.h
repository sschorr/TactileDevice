/* This file controls the 3-DOF wearable device based on the "interaction force" from the
 * CHAI computed device */

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
#include "chai3d.h"
#include <QMutex>

#define PI 3.14159265

// Physical parameters of build [mm] =======================================
// radius of motor pulley
#define MOTRAD 1.5 // [mm]
// horizontal offset of motorshaft from base joint
#define HORIZOFFSET 6.25 // [mm]
// vertical offset of motorshaft from base joint
#define VERTOFFSET -9.5 // [mm]
// distance from base joint to tether attachment point
#define ATTACHL 7.5 // [mm]
// Base joint angles during calibration procedure
#define CALIBANGLE 0.785398 //45 [deg] to rad
// length upper arm
#define L_UA 9 // [mm]
// length lower arm
#define L_LA 9 // [mm]
// length of base (center to joint)
#define L_BASE 15 // [mm]
// length of end effector (center to joint)
#define L_EE 15 // [mm]
// Spring torque of spring at parallel
#define SPRING_TORQUE 0.05 //is the old spring (will use .1 for new motors) [in-.lbs/180 deg]



class c3DOFDevice
{

public:
    // Constructor of c3DOFDevice
    c3DOFDevice();
    ~c3DOFDevice();

    // Instantiates all the motor controllers
    int Init3DOFDeviceEnc();

    // returns the vector of all motor angles (positive torque about joint pushes end effector, angle measured from plane)
    Eigen::Vector3d GetMotorAngles();

    // Determines the base joint angles based on the motor angles (angle measured from plane up to link)
    Eigen::Vector3d GetJointAngles();

    // zeros all of the encoders on each cMotorController
    void ZeroEncoders();

    // gets the end effector position
    Eigen::Vector3d GetCartesianPos();

    // determine the required joint torques for a given force (positive torque about joint pushes end effector)
    Eigen::Vector3d CalcDesiredJointTorques(Eigen::Vector3d);

    // determine the required motor torques for a given force 
    Eigen::Vector3d CalcDesiredMotorTorques(Eigen::Vector3d);

    // determine required motor torques for a given joint torque desired
    Eigen::Vector3d CalcDesiredMotorTorquesJointControl(Eigen::Vector3d);

    // determine the inverse kinematics based on the stored desired position. Gives JOINT angles
    Eigen::Vector3d CalcInverseKinJoint();

    // Set the desired forces
    void SetDesiredForce(Eigen::Vector3d);

    // Set the desired pos (in mm)
    void SetDesiredPos(Eigen::Vector3d);

    // read the desired pos
    Eigen::Vector3d ReadDesiredPos();

    // read the desired forces
    Eigen::Vector3d ReadDesiredForce();

    // set the output Torques for a given cartesian force
    void SetCartesianTorqueOutput(Eigen::Vector3d torqueOutput);

    // set the motor torques for a given desired joint torque
    void SetJointTorqueOutput(Eigen::Vector3d desJointTorqueOutput);

    // read the output voltages
    Eigen::Vector3d ReadVoltageOutput();

    // Commands a position to the 3DOF device
    void PositionController(double Kp, double Kd);

    // Commands joint angles to the motors on the device
    void JointController(double Kp, double Kd);

    // Commands a force to the 3DOF device
    void ForceController();

    // Test torque output of motors
    void TestMotorTorqueController();

    // Set motors to stop outputting
    void TurnOffControl();


    // Public Vars ==========================================
    Eigen::Vector3d neutralPos;
    Eigen::Vector3d desiredForce;
    Eigen::Vector3d desiredPos;
    Eigen::Vector3d jointTorques;
    Eigen::Vector3d motorTorques;


private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;


};

#endif // C3DOFDEVICE_H
