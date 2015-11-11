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



typedef struct
{
    QMutex sharedMutex;
    // This is for any variables that are accessed from multiple threads
    c3DOFDevice* wearableDelta;    

    double GUI_desiredX;
    double GUI_desiredY;
    double GUI_desiredZ;


} shared_data;

#endif // SHARED_DATA_H

