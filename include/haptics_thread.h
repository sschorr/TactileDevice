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
#include "CODE.h"
#include "Win626.h"
#include "windows.h"

#define RANGE_10V 0x00 // Range code for ADC ±10V range.
#define RANGE_5V 0x10 // Range code for ADC ±5V range.
#define EOPL 0x80 // ADC end-of-poll-list marker.
#define CHANMASK 0x0F // ADC channel number mask.

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
    double ComputeContactVibration();
    void SimulateDynamicBodies();
    void InitDynamicBodies();
    void ComputeVRDesiredDevicePos();
    void UpdateVRGraphics();
    void CommandSinPos(Eigen::Vector3d);
    void InitChaiStuff();
    void InitAccel();
    chai3d::cVector3d ReadAccel();

    // clocks
    chai3d::cPrecisionClock rateClock;
    chai3d::cPrecisionClock rateDisplayClock;

    // Public Variables ============================================
    shared_data* p_CommonData; //create a pointer to a shared_data struct

    // chai3D objects
    chai3d::cWorld* world;    
    chai3d::cDirectionalLight* light;
    chai3d::cToolCursor* m_tool0;
    chai3d::cToolCursor* m_tool1;
    chai3d::cShapeSphere* m_curSphere0;  // sphere to hover over tool and enable frames
    chai3d::cShapeSphere* m_curSphere1;
    chai3d::cShapeBox* m_box;    
    chai3d::cMesh* meshBox;
    chai3d::cMesh* ground;
    chai3d::cMultiMesh* finger;
    chai3d::cEffectSurface* newEffect;

    // ODE Module variables
    cODEWorld* ODEWorld;

    // ODE Objects
    cODEGenericBody* ODEBody0;
    cODEGenericBody* ODEBody1;

    // planes confining objects
    cODEGenericBody* ODEGPlane0;
    cODEGenericBody* ODEGPlane1;

    // vars for computed forces
    double toolRadius;
    chai3d::cVector3d lastComputedForce0;
    chai3d::cVector3d magTrackerLastComputedForce0;
    chai3d::cVector3d deviceLastComputedForce0;
    chai3d::cVector3d deviceLastLastComputedForce0;
    chai3d::cVector3d estimatedVel0;

    // tracker rotation variables
    chai3d::cVector3d position0; chai3d::cMatrix3d rotation0;
    chai3d::cMatrix3d fingerRotation0; chai3d::cMatrix3d deviceRotation0;
    chai3d::cVector3d position1; chai3d::cMatrix3d rotation1;
    chai3d::cMatrix3d fingerRotation1; chai3d::cMatrix3d deviceRotation1;

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

    // bandwidth sin variables
    double bandSinAmp;
    double bandSinFreq;

    // device variable
    Eigen::Vector3d neutralPos;

    // variables for handling ADC read in
    BYTE poll_list[16]; // List of items to be digitized.
    SHORT databuf[16]; // Buffer to receive digitized data.

    // accelerometer signals
    chai3d::cVector3d accelSignal;

    DataRecordStruct dataRecorder;

protected:
    void run();

};

#endif // HAPTICS_THREAD_H
