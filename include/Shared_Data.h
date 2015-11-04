// This file defines the shared data structure which is shared with threads
// throughout the program.

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
#define SENSORAY626

#include <qDebug>
#include <iostream>
#include "cMotorController.h"
#include "c3dofdevice.h"



typedef struct
{
    // This is for any variables that are accessed from multiple threads
    c3DOFDevice* wearableDelta;

    // Test variable to be removed later
    cMotorController* testController1;
    cMotorController* testController2;

} shared_data;

#endif // SHARED_DATA_H

