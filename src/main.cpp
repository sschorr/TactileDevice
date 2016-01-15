#include <QApplication>
#include "mainwindow.h"
#include "shared_data.h"
#include "haptics_thread.h"


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
    shared.chaiMagDevice0 = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::c3dofChaiDevice(0)));
    shared.chaiMagDevice1 = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::c3dofChaiDevice(1)));

    // Create a haptic device
    shared.wearableDelta = new c3DOFDevice();
    shared.wearableDelta->Init3DOFDeviceEnc();




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



