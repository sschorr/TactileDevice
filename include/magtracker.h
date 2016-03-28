#ifndef MAGTRACKER_H
#define MAGTRACKER_H

#include "chai3d.h"
#include "TrakSTAR.h"
#include <qDebug.h>

// THIS CODE IS DEPRECATED AND ALL MAG TRACKER FUNCTIONALITY IS IN  magTracker_thread.cpp

class magTracker
{
public:
    magTracker(int trackerNum);
    ~magTracker();

    void InitMagTracker();
    chai3d::cVector3d CheckPos();
    chai3d::cMatrix3d CheckRot();
    chai3d::cTransform CheckPose();

    // magnetic tracker variables
    CSystem     ATC3DG; // a pointer to a single instance of the system class
    CSensor     *pSensor; // a pointer to an array of sensor objects
    CXmtr       *pXmtr; // a pointer to an array of transmitter objects
    CBoard      *pBoard; // a pointer to an array of board objects
    int         errorCode;
    int         sensorID;
    int         transmitterID;
    short       id;
    int         numberBytes;
    int         i;
    int         trackerNum;
    double      measFreq;
    DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD record;
};

#endif // MAGTRACKER_H
