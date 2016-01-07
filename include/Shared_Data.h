// This file defines the shared data structure which is shared with threads
// throughout the program.

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
#define SENSORAY626

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

// define the data structure that holds stored data
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

} DataRecordStruct;

// define the data structure that contains shared data
typedef struct
{
    //Chai3D variables
    chai3d::cCamera* p_camera;

    QMutex sharedMutex;

    c3DOFDevice* wearableDelta;
    chai3d::cGenericHapticDevicePtr chaiDevice; // a pointer to the current haptic device

    bool posControlMode;
    bool hapticsThreadActive;

    double hapticRateEstimate;

    std::vector<DataRecordStruct> debugData;

} shared_data;




#endif // SHARED_DATA_H

