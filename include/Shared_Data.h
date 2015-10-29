#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#define TACTORCENTER 2.3
#define CAPSTAN_RADIUS_IN_MM 4.75

#define PART_A 'A'
#define PART_B 'B'
#define PART_C 'C'
#define PART_D 'D'
#define PART_E 'E'
#define PART_F 'F'
#define PART_G 'G'


#define DEVICE_NAME_1 "Phantom1"
#define DEVICE_NAME_2 "Phantom2"


#define ACTIVATE_SS_DEVICE
#define ACTIVATE_PHANTOMS
//#define ACTIVATE_FORCE_SENSOR


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

#include <cstdio>
#include <cassert>
#include <windows.h>
#include <stdio.h>
#include <queue>
#include "KinectDevice.h"

#include "chai3d.h"
#include "CShearStylus.h"
#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include "SimpleIni.h"

#include "cForceSensor.h"

typedef enum
{
    ExperimentRunningTrial,
    ExperimentRunningLine,
    Idle,
    StartUp,
    Break,
    End

} sm_states;

typedef struct
{
    hduVector3Dd pos_1;
    hduVector3Dd pos_2;
    hduVector3Dd vel_1;
    hduVector3Dd vel_2;
    hduVector3Dd pos_1delay;
    hduVector3Dd pos_2delay;
    hduVector3Dd vel_1delay;
    hduVector3Dd vel_2delay;
    hduVector3Dd filtered_vel_1;
    hduVector3Dd filtered_vel_2;
    hduVector3Dd filtered_vel_1delay;
    hduVector3Dd filtered_vel_2delay;
    hduVector3Dd force_1;
    hduVector3Dd force_2;


    double SSPos;
    double SSPosDesired;
    double SSForce;
    double Time;

    double ForceSensor_x;
    double ForceSensor_y;
    double ForceSensor_z;

    double TorqueSensor_x;
    double TorqueSensor_y;
    double TorqueSensor_z;

} DataRecorder;

typedef struct
{
    cShearStylus* p_ShearStylus;
    KinectDevice* p_kinect;

    HHD phantomId_1, phantomId_2;

    //skin stretch variables
    double TactorPosDesired;
    double SSG_pos;
    double TactorPos;
    double TactorPos_Last;
    double SkinStretchDeviceForce;
    double delay;
    double hapticTimeoutRate;


    hduVector3Dd vel_1;
    hduVector3Dd vel_2;
    hduVector3Dd pos_1;
    hduVector3Dd pos_2;
    hduVector3Dd vel_1delay;
    hduVector3Dd vel_2delay;
    hduVector3Dd pos_1delay;
    hduVector3Dd pos_2delay;
    hduVector3Dd forceVec_1, forceVec_2, forceVec_3, forceVec_4;
    hduVector3Dd motor_temp_1;
    hduVector3Dd motor_temp_2;

    double VerticalForceSlave;

    double LineAngle;

    int rateDisplayCounter;
    int rateEstimate;

    char controller;
    int trial;
    int maxTrial;
    sm_states experimentState;

    bool recordFlag;
    bool writeToFileFlag;
    bool recordingTrial;
    bool recordingLine;
    bool startTimeFlag;
    bool filterFlag;
    bool cameraFlag;
    bool vibrationFlag;
    bool forceFeedbackFlag;
    bool skinStretchFlag;
    bool barFlag;
    bool reducedGains;
    bool breakFlag;
    bool endFlag;
    int trialAngle;
    bool premiumsOn;

    QString dir;
    QString fileName;
    QString trialNumber;
    QString protocolLocation;
    CSimpleIniA protocolFile;

    std::vector<DataRecorder> DataVectorTrial;
    std::vector<DataRecorder> DataVectorLine;


} shared_data;

#endif // SHARED_DATA_H
