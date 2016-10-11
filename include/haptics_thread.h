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
#include <qDebug>
#include <string>
#include <QString>
#include <ostream>
#include <istream>
#include <fstream>
#include <iostream>
#include <sstream>
#include "windows.h"
#include <time.h>


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
    void ComputeVRDesiredDevicePos();
    void UpdateVRGraphics();
    void CommandSinPos(Eigen::Vector3d);
    void CommandCircPos(Eigen::Vector3d);

    void InitGeneralChaiStuff();
    void InitFingerAndTool();
    void InitEnvironments();
    void RenderPalpation();
    void RenderTwoFriction();
    void RenderHump();
    void RenderExpFriction();
    void WriteDataToFile();
    void InitDynamicBodies();
    void RenderDynamicBodies();
    void SetInitJointAngles();
    void UpdateScaledBoxes();
    void UpdateScaledCursors();
    void UpdateScaledFingers();
    void rotateTissueLineDisp(double angle);
    void rotateTissueLine(double angle);
    void AddImpulseDisp(chai3d::cVector3d &indexForce, chai3d::cVector3d &thumbForce);
    void AddImpulseTorqueDisp(chai3d::cVector3d &indexForce, chai3d::cVector3d &thumbForce);

    // clocks
    chai3d::cPrecisionClock rateClock;
    chai3d::cPrecisionClock rateDisplayClock;

    double currTime;
    double timeInterval;
    double lastTime;

    // Public Variables ============================================
    shared_data* p_CommonData; //create a pointer to a shared_data struct

    // chai3D objects
    chai3d::cWorld* world;    
    chai3d::cDirectionalLight* light;
    chai3d::cToolCursor* m_tool0;
    chai3d::cToolCursor* m_tool1;
    chai3d::cShapeSphere* m_curSphere0;  // sphere to hover over tool and enable frames
    chai3d::cShapeSphere* m_curSphere1;
    chai3d::cMesh* ground;
    chai3d::cMultiMesh* finger;
    chai3d::cMultiMesh* thumb;
    chai3d::cEffectSurface* newEffect;
    cODEGenericBody* ODEGPlane0;
    cODEGenericBody* ODEGPlane1;
    chai3d::cMesh* globe;    

    // objects for doing display scaling
    chai3d::cMultiMesh* scaledFinger;
    chai3d::cMultiMesh* scaledThumb;

    chai3d::cShapeSphere* m_dispScaleCurSphere0;
    chai3d::cShapeSphere* m_dispScaleCurSphere1;

    // offsets for haptic tool from finger models
    chai3d::cVector3d fingerOffset;
    chai3d::cVector3d thumbOffset;

    // initial position of boxes (center of warping for display scaled boxes)
    chai3d::cVector3d box1InitPos;
    chai3d::cVector3d box2InitPos;
    chai3d::cVector3d box3InitPos;

    //texture for globe
    chai3d::cTexture2dPtr textureSpace;

    double boxSize;
    double groundSize;

    // ODE Module variables
    cODEWorld* ODEWorld;

    // vars for storing computed impulse forces
    chai3d::cVector3d indexImpulse;
    chai3d::cVector3d thumbImpulse;
    chai3d::cVector3d indexTorqueImpulse;
    chai3d::cVector3d thumbTorqueImpulse;

    // vars for computed forces
    double toolRadius;
    chai3d::cVector3d filteredDeviceForce0;
    chai3d::cVector3d computedForce0;
    chai3d::cVector3d deviceComputedForce0;
    Eigen::Vector3d deviceForceRecord0;
    Eigen::Vector3d globalForceRecord0;
    chai3d::cVector3d lastFilteredDeviceForce0;
    chai3d::cVector3d estimatedVel0;

    chai3d::cVector3d filteredDeviceForce1;
    chai3d::cVector3d computedForce1;
    chai3d::cVector3d deviceComputedForce1;
    Eigen::Vector3d deviceForceRecord1;
    Eigen::Vector3d globalForceRecord1;
    chai3d::cVector3d lastFilteredDeviceForce1;
    chai3d::cVector3d estimatedVel1;

    // tracker rotation variables
    chai3d::cVector3d position0; chai3d::cMatrix3d rotation0;
    chai3d::cMatrix3d fingerRotation0; chai3d::cMatrix3d deviceRotation0;

    chai3d::cVector3d position1; chai3d::cMatrix3d rotation1;
    chai3d::cMatrix3d fingerRotation1; chai3d::cMatrix3d deviceRotation1;

    // ints for display counters
    int rateDisplayCounter;
    int recordDataCounter;

    // variables for handling ADC read in
    BYTE poll_list[16]; // List of items to be digitized.
    SHORT databuf[16]; // Buffer to receive digitized data.

protected:
    void run();

};

#endif // HAPTICS_THREAD_H
