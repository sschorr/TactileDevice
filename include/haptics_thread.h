#ifndef HAPTICS_THREAD_H
#define HAPTICS_THREAD_H

#include <QObject>
#include <QThread>
#include "shared_data.h"
#include "c3DOFdevice.h"
#include "chai3d.h"
#include <stdio.h>
#include <QDir>
#include <QCoreApplication>



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
    chai3d::cShapeSphere* m_curSphere;  // sphere to hover over tool and enable frames
    chai3d::cMesh* meshBox;
    chai3d::cMultiMesh* finger;
    chai3d::cEffectSurface* newEffect;
    double toolRadius;
    chai3d::cVector3d lastComputedForce;
    chai3d::cVector3d magTrackerLastComputedForce;
    chai3d::cVector3d deviceLastComputedForce;
    chai3d::cVector3d deviceLastLastComputedForce;
    chai3d::cVector3d estimatedVel;

    //vars for contact vibration
    bool firstTouch;
    double decaySinTime;
    double decaySinExp;
    double decaySinAmp;
    double decaySinScale;
    double decaySinAmpMax;
    double decaySinFreq;
    double computedPosAdd;




    // ints for display counters
    int rateDisplayCounter;
    int recordDataCounter;

    DataRecordStruct dataRecorder;



protected:
    void run();

};

#endif // HAPTICS_THREAD_H
