#include <QApplication>
#include "mainwindow.h"
#include "shared_data.h"
#include "haptics_thread.h"
#include "experiment_thread.h"
#include "magTracker_thread.h"

// Variable Declarations ============================================
shared_data shared; //create the shared_data structure for sharing

// Testing Variable Declaration =====================================

// Function Declarations ============================================

// Create Threads ===================================================
haptics_thread hapticsThread;
Experiment_Thread experimentThread;
magTracker_thread magTrackerThread;


// MAIN FUNCTION ====================================================
int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow window;

    // Create a chai device for mag tracker
    shared.chaiMagDevice0 = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::c3dofChaiDevice(0)));
    shared.chaiMagDevice1 = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::c3dofChaiDevice(1)));

    // Create a haptic device
    shared.wearableDelta = new c3DOFDevice();
    shared.wearableDelta->Init3DOFDeviceEnc();

    // Initialize and set the haptics thread data pointer to the shared data
    hapticsThread.p_CommonData = &shared;
    experimentThread.p_CommonData = &shared;
    magTrackerThread.p_CommonData = &shared;

    // Initialize the threads
    hapticsThread.initialize();
    experimentThread.initialize();
    magTrackerThread.initialize();

    // Initialize and set the window thread data pointer to the shared data
    window.p_CommonData = &shared;
    window.show();
    window.Initialize();

    hapticsThread.start();
    experimentThread.start();
    magTrackerThread.start();

    return a.exec();
}



