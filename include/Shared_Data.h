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
#include "c3dofdevice.h"
#include <Eigen/Dense>

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
    QMutex sharedMutex;
    // This is for any variables that are accessed from multiple threads
    c3DOFDevice* wearableDelta;

    bool forceControlMode;
    bool posControlMode;
    bool hapticsThreadActive;

    double hapticRateEstimate;

    std::vector<DataRecordStruct> debugData;

} shared_data;




#endif // SHARED_DATA_H

