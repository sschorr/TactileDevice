#ifndef MAGTRACKER_THREAD_H
#define MAGTRACKER_THREAD_H

#include <QObject>
#include <QThread>
#include "chai3d.h"
#include <qDebug.h>
#include "Shared_Data.h"
#include <QTimer>

class magTracker_thread : public QThread
{
    Q_OBJECT

public:
    explicit magTracker_thread(QObject *parent = 0);
    ~magTracker_thread();

    void initialize();
    void CheckTrackerPoses();

    shared_data* p_CommonData;

    chai3d::cPrecisionClock runTimer;

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



protected:
    void run();
};

#endif // magTracker_thread_H
