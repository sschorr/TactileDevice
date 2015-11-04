#include <QApplication>
#include "mainwindow.h"
#include "shared_data.h"
#include "haptics_thread.h"


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


    // Create a haptic device
    shared.wearableDelta = new c3DOFDevice();
    shared.wearableDelta->Init3DOFDeviceEnc();


    // Initialize and then start the haptics thread
    hapticsThread.p_CommonData = &shared; // set the haptics thread data pointer to the shared data
    hapticsThread.initialize();
    hapticsThread.start();

    window.p_CommonData = &shared;
    window.Initialize();

    window.show();
    return a.exec();
}



