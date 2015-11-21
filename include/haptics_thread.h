#ifndef HAPTICS_THREAD_H
#define HAPTICS_THREAD_H

#include <QObject>
#include <QThread>
#include "shared_data.h"
#include "c3dofdevice.h"
#include "chai3d.h"
#include <stdio.h>


class haptics_thread : public QThread
{
    Q_OBJECT


public:
    // Public Functions ============================================
    explicit haptics_thread(QObject *parent = 0);
    ~haptics_thread();
    void initialize();
    void RecordData();

    // clocks
    chai3d::cPrecisionClock rateClock;
    chai3d::cPrecisionClock rateDisplayClock;
    chai3d::cPrecisionClock overallClock;


    // Public Variables ============================================
    shared_data* p_CommonData; //create a pointer to a shared_data struct

    int rateDisplayCounter;
    int recordDataCounter;

    DataRecordStruct dataRecorder;



protected:
    void run();

};

#endif // HAPTICS_THREAD_H
