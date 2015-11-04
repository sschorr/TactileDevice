// This file defines the shared data structure which is shared with threads
// throughout the program.

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
#define SENSORAY626

#include <qDebug>
#include <iostream>
#include <QVector>
#include <math.h>
#include "cMotorController.h"
#include "c3dofdevice.h"




typedef struct
{
    // This is for any variables that are accessed from multiple threads
    c3DOFDevice* wearableDelta;

} shared_data;

#endif // SHARED_DATA_H

