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

// Physical parameters of build agnostic [mm] =======================================
#define SPRING_TORQUE 0.1               // 0.1 for stiff, .05 for weak [in-.lbs/180 deg], measured at parallel
#define MOTRAD 1.5                      // radius of motor pulley [mm]
#define HORIZOFFSET 6.25                // horizontal offset of motorshaft from base joint [mm]
#define VERTOFFSET -9.5                 // vertical offset of motorshaft from base joint [mm]
#define CALIBANGLE 0.785398             // Base joint angles during calibration procedure (45 [deg] to rad)
// Physical parameters of build index [mm] =======================================
#define ATTACHL_0 7.5                   // distance from base joint to tether attachment point [mm]
#define L_UA_0 9                        // length upper arm [mm]
#define L_LA_0 9                        // length lower arm [mm]
#define L_BASE_0 15                     //length of base (center to joint)[mm]
#define L_EE_0 15                       // length of end effector (center to joint)[mm]
// Physical parameters of build thumb [mm] =======================================
#define ATTACHL_1 10.5                  // distance from base joint to tether attachment point [mm]
#define L_UA_1 9                        // length upper arm [mm]
#define L_LA_1 12                       // length lower arm [mm]
#define L_BASE_1 15                     //length of base (center to joint)[mm]
#define L_EE_1 18                       // length of end effector (center to joint)[mm]


class c3DOFDevice
{

public:
    // Constructor of c3DOFDevice
    c3DOFDevice(int num);
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

    // allows setting individual joints on startup
    void IndivJointController(Eigen::Vector3d desJointAnglesArg, double Kp, double Kd);


    // Public Vars ==========================================
    Eigen::Vector3d neutralPos;
    Eigen::Vector3d desiredForce;
    Eigen::Vector3d desiredPos;
    Eigen::Vector3d jointTorques;
    Eigen::Vector3d motorTorques;

    // Physical parameters of build thumb assigned based on whether index or thumb in init
    double ATTACHL;                  // distance from base joint to tether attachment point [mm]
    double L_UA;                        // length upper arm [mm]
    double L_LA ;                       // length lower arm [mm]
    double L_BASE;                     //length of base (center to joint)[mm]
    double L_EE;                       // length of end effector (center to joint)[mm]

private:
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;

    int finger;
};

#endif // C3DOFDEVICE_H
