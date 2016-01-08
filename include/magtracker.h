#ifndef MAGTRACKER_H
#define MAGTRACKER_H

#include "chai3d.h"
#include "TrakSTAR.h"
#include <qDebug.h>

class magTracker
{
public:
    magTracker();
    ~magTracker();

    void InitMagTracker();
    chai3d::cVector3d CheckPos();
    chai3d::cMatrix3d CheckRot();

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
    double measFreq;
    DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD record;
    DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD* pRecord;
};

#endif // MAGTRACKER_H
