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

#define E_VALUE 2.718

// define the data structure that holds stored data
typedef struct
{
    double time;
    chai3d::cVector3d pos;
    chai3d::cVector3d desiredPos;
    chai3d::cVector3d motorAngles;
    chai3d::cVector3d jointAngles;
    chai3d::cVector3d motorTorque;
    chai3d::cVector3d voltageOut;
    chai3d::cVector3d desiredForce;

} DataRecordStruct;

// define the data structure that contains shared data
typedef struct
{
    //Chai3D variables
    chai3d::cCamera* p_camera;

    QMutex sharedMutex;

    c3DOFDevice* wearableDelta;
    chai3d::cGenericHapticDevicePtr chaiMagDevice0; // a pointer to the current haptic device
    chai3d::cGenericHapticDevicePtr chaiMagDevice1;

    bool posControlMode;
    bool hapticsThreadActive;

    double hapticRateEstimate;

    std::vector<DataRecordStruct> debugData;

} shared_data;




#endif // SHARED_DATA_H

