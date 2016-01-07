#include <QApplication>
#include "mainwindow.h"
#include "shared_data.h"
#include "haptics_thread.h"
#include "TrakSTAR.h"

// This code runs the pololu motor driven device stably


// Variable Declarations ============================================
shared_data shared; //create the shared_data structure for sharing

// Testing Variable Declaration =====================================

// Function Declarations ============================================

// Create Threads ===================================================
haptics_thread hapticsThread;


// MAIN FUNCTION ====================================================
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow window;

    // Create a chai device
    shared.chaiDevice = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::c3dofChaiDevice()));

    // Create a haptic device
    shared.wearableDelta = new c3DOFDevice();
    shared.wearableDelta->Init3DOFDeviceEnc();

    // TESTING WITH MAGNETIC TRACKER
    /*CSystem ATC3DG; // a pointer to a single instance of the system class
    CSensor *pSensor; // a pointer to an array of sensor objects
    CXmtr *pXmtr; // a pointer to an array of transmitter objects
    CBoard *pBoard; // a pointer to an array of board objects

    int errorCode;
    int sensorID;
    int transmitterID;
    short id;
    int numberBytes;
    int i = 0;

    DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD record, *pRecord = &record;

    // initialize the magnetic tracker
    qDebug("Initializing the ATC3DG system...\n");
    errorCode = InitializeBIRDSystem();
    if(errorCode==BIRD_ERROR_SUCCESS){
        qDebug("Initialized ATC3DG system\n");
    }
    // get configurations
    errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
    pSensor = new CSensor[ATC3DG.m_config.numberSensors];
    for(i=0;i<ATC3DG.m_config.numberSensors;i++)
    {
        errorCode = GetSensorConfiguration(i, &pSensor[i].m_config);
        if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
        qDebug("Got sensors configuration\n");
    }
    pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
    for(i=0;i<ATC3DG.m_config.numberTransmitters;i++)
    {
        errorCode = GetTransmitterConfiguration(i, &pXmtr[i].m_config);
        if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
        qDebug("Got transmitters configuration\n");
    }*/


    // Initialize and set the haptics thread data pointer to the shared data
    hapticsThread.p_CommonData = &shared;
    hapticsThread.initialize();

    // Initialize and set the window thread data pointer to the shared data
    window.p_CommonData = &shared;
    window.Initialize();

    hapticsThread.start();
    window.show();

    return a.exec();
}



