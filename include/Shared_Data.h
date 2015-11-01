// This file defines the shared data structure which is shared with threads
// throughout the program.

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <qDebug>
#include "cMotorController.h"


typedef struct
{
    cMotorController* motor_1;
    cMotorController* motor_2;
    cMotorController* motor_3;

} shared_data;

#endif // SHARED_DATA_H

