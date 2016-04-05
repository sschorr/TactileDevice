// This file defines the shared data structure which is shared with threads
// throughout the program.
#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// defines indicating what physical hardware is present
//#define SENSORAY626
//#define MAGTRACKER

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
#include <Qt/qfiledialog.h>
#include <Qt/qinputdialog.h>
#include "CODE.h"

#define POS_SCALE 6
#define HAPTIC_X_TRANSLATE 0 //.12
#define HAPTIC_Y_TRANSLATE 0
#define HAPTIC_Z_TRANSLATE 0 // -0.05

#define CAMERA_X_TRANSLATE 0 //0.15
#define CAMERA_Y_TRANSLATE 0.00
#define CAMERA_Z_TRANSLATE 0

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

} DataRecordStruct;

typedef enum
{
    idleExperiment,
    frictionTrial,
    palpationTrial,
    trialBreak,
    end
} experiment_states;

typedef enum
{
    idleControl,
    neutralPosControl,
    sliderControlMode,
    VRControlMode,
    sinControlMode,
    circControlMode    
} control_states;

typedef enum
{
    none,
    palpation,
    friction,
    hump,
    hoopHump,
    experimentFriction,
    experimentPalpation,
    dynamicBodies,
    paperEnvironment
} environment_states;


// define the data structure that contains data that we share between threads
typedef struct
{
    //Chai3D variables

    // camera vars
    chai3d::cCamera* p_camera;
    chai3d::cVector3d cameraPos;
    chai3d::cVector3d lookatPos;
    chai3d::cVector3d upVector;

    // camera vars for polar camera movement
    double azimuth;
    double polar;
    double camRadius;

    QMutex sharedMutex;

    // the delta mechanism class
    c3DOFDevice* wearableDelta;

    // the delta mechanism chai class
    chai3d::cGenericHapticDevicePtr chaiMagDevice0; // a pointer to the current haptic device
    chai3d::cGenericHapticDevicePtr chaiMagDevice1; // support two mag sensors

    //clock for all threads
    chai3d::cPrecisionClock overallClock;

    //clock for showing temp transparent, then moving to next trial
    chai3d::cPrecisionClock palpPostTrialClock;

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

    // joint controller variables
    double jointKp;
    double jointKd;

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


    // updatable neutral position
    Eigen::Vector3d neutralPos;

    // protocol loading
    QString frictionProtocolLocation;
    QString palpationProtocolLocation;
    CSimpleIniA frictionProtocolFile;
    CSimpleIniA palpationProtocolFile;

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
    chai3d::cMultiMesh* p_indicator;

    // haptics thread objects for friction environment
    chai3d::cMesh* p_frictionBox1;
    chai3d::cMesh* p_frictionBox2;

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

    // random positions for lump in tissue
    double xRandPos;
    double yRandPos;

    // haptics thread objects for Dynamic (ODE) environments
    cODEGenericBody* ODEBody0;
    cODEGenericBody* ODEBody1;
    // haptics thread objects for visual representation of dynamic objects
    chai3d::cMesh* p_dynamicBox;

    // planes confining objects
    cODEGenericBody* ODEGPlane0;
    cODEGenericBody* ODEGPlane1;

    // haptics thread objects for hump environment
    chai3d::cMultiMesh* p_hump;
    chai3d::cMultiMesh* p_hoopHump;

    bool environmentChange;
    bool m_flagTissueTransparent;

    // recording variables
    QString dir;
    QString fileName;

} shared_data;




#endif // SHARED_DATA_H

