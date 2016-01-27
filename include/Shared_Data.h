// This file defines the shared data structure which is shared with threads
// throughout the program.

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
//#define SENSORAY626
//#define MAGTRACKER

#include <qDebug>
#include <QVector>
#include <math.h>
#include "cMotorController.h"
#include "c3dofChaiDevice.h"
#include "c3DOFdevice.h"
#include <Eigen/Dense>
#include "chai3d.h"

#define POS_SCALE 6
#define HAPTIC_X_TRANSLATE 0 //.12
#define HAPTIC_Y_TRANSLATE 0
#define HAPTIC_Z_TRANSLATE 0 // -0.05

#define CAMERA_X_TRANSLATE 0 //0.15
#define CAMERA_Y_TRANSLATE 0.00
#define CAMERA_Z_TRANSLATE 0

#define STIFFNESS_BASELINE 50
#define STIFFNESS_INCREMENT 10
#define STATIC_FRICTION 1
#define DYNAMIC_FRICTION 1

#define E_VALUE 2.718

// define the data structure that holds recorded data
typedef struct
{
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d desiredPos;
    Eigen::Vector3d motorAngles;
    Eigen::Vector3d jointAngles;
    Eigen::Vector3d motorTorque;
    Eigen::Vector3d voltageOut;
    Eigen::Vector3d desiredForce;
    chai3d::cVector3d magTrackerPos;
    chai3d::cVector3d accelSignal;

} DataRecordStruct;

typedef enum
{
    idle,
    sliderControlMode,
    VRControlMode,
    sinControlMode

} sm_states;


// define the data structure that contains data that we share between threads
typedef struct
{
    //Chai3D variables
    chai3d::cCamera* p_camera;
    QMutex sharedMutex;
    c3DOFDevice* wearableDelta;
    chai3d::cGenericHapticDevicePtr chaiMagDevice0; // a pointer to the current haptic device
    chai3d::cGenericHapticDevicePtr chaiMagDevice1; // support two mag sensors

    //clock for all threads
    chai3d::cPrecisionClock overallClock;

    // determine start time for bandwidth sin
    double sinStartTime;

    // Declare the state that we are in
    sm_states currentState;

    // check whether to record
    bool recordFlag;

    bool hapticsThreadActive;
    double hapticRateEstimate;
    std::vector<DataRecordStruct> debugData;

    // bandwidth sin variables
    double bandSinAmp;
    double bandSinFreq;

    // controller variables
    double Kp;
    double Kd;

    // haptics thread objects that window thread needs access too
    chai3d::cMultiMesh* p_table;
    chai3d::cMultiMesh* p_petriDish;
    chai3d::cMultiMesh* p_tissueOne;
    chai3d::cMultiMesh* p_tissueTwo;
    chai3d::cMultiMesh* p_tissueThree;
    chai3d::cMultiMesh* p_tissueFour;
    chai3d::cMultiMesh* p_tissueFive;
    chai3d::cMultiMesh* p_tissueSix;
    chai3d::cMultiMesh* p_tissueSeven;
    chai3d::cMultiMesh* p_indicator;

    bool m_flagTissueTransparent;

} shared_data;




#endif // SHARED_DATA_H

