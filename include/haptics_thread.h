#ifndef HAPTICS_THREAD_H
#define HAPTICS_THREAD_H

#include <QObject>
#include <QThread>
#include "shared_data.h"
#include "c3DOFdevice.h"
#include "chai3d.h"
#include <stdio.h>


class haptics_thread : public QThread
{
    Q_OBJECT


public:
    // Public Functions ============================================

    // CONSTRUCTOR
    explicit haptics_thread(QObject *parent = 0);

    // DESTRUCTOR
    ~haptics_thread();

    // METHODS
    void initialize();
    void RecordData();

    // clocks
    chai3d::cPrecisionClock rateClock;
    chai3d::cPrecisionClock rateDisplayClock;
    chai3d::cPrecisionClock overallClock;


    // Public Variables ============================================
    shared_data* p_CommonData; //create a pointer to a shared_data struct

    // chai3D objects
    chai3d::cWorld* world;
    chai3d::cDirectionalLight* light;
    chai3d::cToolCursor* m_tool;
    chai3d::cShapeBox* m_box;
    chai3d::cMesh* meshBox;
    chai3d::cEffectSurface* newEffect;
    double toolRadius;

    chai3d::cVector3d lastComputedForce;

    int rateDisplayCounter;
    int recordDataCounter;

    DataRecordStruct dataRecorder;



protected:
    void run();

};

#endif // HAPTICS_THREAD_H
