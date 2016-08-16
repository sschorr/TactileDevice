// This file defines the shared data structure which is shared with threads
// throughout the program.
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
//#define MAGTRACKER
//#define SENSORAY826
//#define OCULUS
//#define QWT

#include <qDebug>
#include <QVector>
#include <math.h>
#include "cMotorController.h"
#include "c3dofChaiDevice.h"
#include "c3DOFdevice.h"
#include <Eigen/Dense>
#include "chai3d.h"
#include "SimpleIni.h"
#include <QDebug>
#include <QMetaType>
#include <QString>
#include <string>
#include <iostream>
#include <ostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <vector>
#include <qfiledialog>
#include <qinputdialog>
#include "CODE.h"

#define STIFFNESS_BASELINE 50
#define STIFFNESS_INCREMENT 10
#define STATIC_FRICTION 1
#define DYNAMIC_FRICTION 1

#define E_VALUE 2.718

// define the data structure that holds recorded data
typedef struct
{
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d desiredPos;
    Eigen::Vector3d motorAngles;
    Eigen::Vector3d jointAngles;
    Eigen::Vector3d motorTorque;
    Eigen::Vector3d voltageOut;
    Eigen::Vector3d VRInteractionForce;
    Eigen::Vector3d VRInteractionForceGlobal;
    chai3d::cVector3d magTrackerPos0;
    chai3d::cVector3d magTrackerPos1;
    chai3d::cVector3d accelSignal;
    int tactileFeedback;
    int referenceFirst;
    int pairNo;
    double referenceFriction;
    double comparisonFriction;
    int subjectAnswer;
    chai3d::cVector3d lumpLocation;
    chai3d::cVector3d lumpAnswerLocation;
    double lineAngle;
    double lineAngleTruth;
    chai3d::cMatrix3d deviceRotation;

} DataRecordStruct;

typedef enum
{
    idleExperiment,
    frictionTrial,
    palpationTrial,
    palpationLineTrial,
    palpationLineWritingToFile,
    palpationLineBreak,
    trialBreak,
    endExperiment
} experiment_states;

typedef enum
{
    idleControl,
    neutralPosControl,
    sliderControlMode,
    VRControlMode,
    sinControlMode,
    circControlMode,
    initCalibControl
} control_states;

typedef enum
{
    none,
    palpation,
    twoFriction,
    hump,
    hoopHump,
    experimentFriction,
    experimentPalpationLine,
    dynamicBodies,
    paperEnvironment
} environment_states;

typedef enum
{
    standard,
    mass,
    friction,
    dimension,
    stiffness
} dynamicObject_states;


// define the data structure that contains data that we share between threads
typedef struct
{
    //Chai3D variables

    // camera vars
    chai3d::cCamera* p_camera;
    chai3d::cWorld * p_world;
    chai3d::cVector3d cameraPos;
    chai3d::cVector3d lookatPos;
    chai3d::cVector3d upVector;

    // camera vars for polar camera movement
    double azimuth;
    double polar;
    double camRadius;

    QMutex sharedMutex;

    // the delta mechanism class
    c3DOFDevice* wearableDelta0;
    c3DOFDevice* wearableDelta1;

    // the delta mechanism chai class
    chai3d::cGenericHapticDevicePtr chaiMagDevice0; // a pointer to the current haptic device
    chai3d::cGenericHapticDevicePtr chaiMagDevice1; // support two mag sensors

    //clock for all threads
    chai3d::cPrecisionClock overallClock;

    //clock for showing temp transparent, then moving to next trial
    chai3d::cPrecisionClock palpPostTrialClock;

    //clock for doing the startup trajectory
    chai3d::cPrecisionClock calibClock;

    // determine start time for bandwidth sin
    double sinStartTime;

    // determine start time for circ
    double circStartTime;

    // Declare the control state that we are in
    control_states currentControlState;

    // Declare the environment state that we are in
    environment_states currentEnvironmentState;

    // The experiment state we are in
    experiment_states currentExperimentState;

    // Which one of the dynamic object states we are in
    dynamicObject_states currentDynamicObjectState;

    // circle drawing radius
    double circRadius;

    // check whether to record
    bool recordFlag;

    bool hapticsThreadActive;
    double hapticRateEstimate;

    // have window thread tell haptics thread to render transparent temporarily
    bool tempTransparentFlag;

    // our data storage variable
    std::vector<DataRecordStruct> debugData;

    // bandwidth sin variables
    double bandSinAmpDisp;
    double bandSinFreqDisp;
    double bandSinAmp;
    double bandSinFreq;

    // position controller variables
    double Kp;
    double Kd;

    // vars for plotting real time force and filtered force from haptics thread
    chai3d::cVector3d deviceComputedForce;
    chai3d::cVector3d filteredDeviceComputedForce;

    // joint controller variables
    double jointKp;
    double jointKd;

    // device initing flags
    bool device0Initing;
    bool device1Initing;

    // the trial number of the experiment
    int trialNo;
    // are we doing the first or second of the comparison
    int pairNo;
    // Is this trial reference or comparison first
    bool referenceFirst;
    // for reading in the friction values
    double comparisonFriction;
    double referenceFriction;
    // for reading in the experiment feedback condition
    int tactileFeedback;

    // answer to which was stiffer
    int subjectAnswer;

    // protocol loading
    QString frictionProtocolLocation;
    QString palpationProtocolLocation;
    QString palpationLineProtocolLocation;
    CSimpleIniA frictionProtocolFile;
    CSimpleIniA palpationProtocolFile;
    CSimpleIniA palpationLineProtocolFile;

    // haptics thread objects for palpation environment
    chai3d::cMultiMesh* p_table;
    chai3d::cMultiMesh* p_petriDish;
    chai3d::cMultiMesh* p_tissueOne;
    chai3d::cMultiMesh* p_tissueTwo;
    chai3d::cMultiMesh* p_tissueThree;
    chai3d::cMultiMesh* p_tissueFour;
    chai3d::cMultiMesh* p_tissueFive;
    chai3d::cMultiMesh* p_tissueSix;
    chai3d::cMultiMesh* p_tissueSeven;
    chai3d::cMultiMesh* p_tissueEight;
    chai3d::cMultiMesh* p_tissueNine;
    chai3d::cMultiMesh* p_tissueTen;
    chai3d::cMultiMesh* p_tissueEleven;
    chai3d::cMultiMesh* p_tissueTwelve;
    chai3d::cMultiMesh* p_indicator;

    // haptics thread objects for friction environment
    chai3d::cMesh* p_frictionBox1; //mesh for left friction surface
    chai3d::cMesh* p_frictionBox2; //mesh for right friction surface

    // haptics thread objects for friction experiment
    chai3d::cMesh* p_expFrictionBox;

    // haptics thread objects for palpation experiment
    chai3d::cMesh* p_tissueCyl;
    chai3d::cMesh* p_tissueLump;
    chai3d::cMesh* p_tissueLumpCenter;
    chai3d::cMesh* p_tissueLumpCenter1;
    chai3d::cMesh* p_tissueLumpCenter2;
    chai3d::cMesh* p_tissueLumpCenter3;
    chai3d::cMesh* p_tissueBox;

    // rotation of line in tissue
    double tissueRot;
    double indicatorRot;

    // haptics thread objects for Dynamic (ODE) environments
    cODEGenericBody* ODEBody1; //ODE body for box 1
    cODEGenericBody* ODEBody2; //ODE body for box 2
    cODEGenericBody* ODEBody3; //ODE body for box 3

    // haptics thread objects for visual representation of dynamic objects
    chai3d::cMesh* p_dynamicBox1; // mesh for box 1
    chai3d::cMesh* p_dynamicBox2; // mesh for box 2
    chai3d::cMesh* p_dynamicBox3; // mesh for box 3

    // flags for environment change and tissue transparency
    bool environmentChange;
    bool m_flagTissueTransparent;

    // recording variables
    QString dir;
    QString fileName;

    // init joint angles
    Eigen::Vector3d desJointInits0;
    Eigen::Vector3d desJointInits1;

    // flags for only normal or only lateral control
    bool flagLateral;
    bool flagNormal;

} shared_data;




#endif // SHARED_DATA_H

