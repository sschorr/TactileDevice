#include "haptics_thread.h"

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::run()
{
    while(p_CommonData->hapticsThreadActive)
    {
        // if clock controlling haptic rate times out
        if(rateClock.timeoutOccurred())
        {
            // stop clock while we perform haptic calcs
            rateClock.stop();
            if(p_CommonData->forceControlMode == true)
            {
                p_CommonData->wearableDelta->ForceController();
            }

            else if(p_CommonData->posControlMode == true)
            {
                p_CommonData->wearableDelta->PositionController();
            }

            // update our rate estimate every second
            rateDisplayCounter++;
            if(rateDisplayClock.timeoutOccurred())
            {
                rateDisplayClock.stop();
                p_CommonData->hapticRateEstimate = rateDisplayCounter;
                rateDisplayCounter = 0;
                rateDisplayClock.start(true);
            }


            // record only on every 10 haptic loops
            recordDataCounter++;            
            if(recordDataCounter == 10)
            {
                RecordData();
            }


            //restart the rateClock
            rateClock.start(true);
        }        
    }

    // If we are terminating, delete the haptic device to set outputs to 0
    delete p_CommonData->wearableDelta;
}

void haptics_thread::initialize()
{    
    // Ensure the device is not controlling to start
    p_CommonData->forceControlMode = false;
    p_CommonData->posControlMode = false;

    p_CommonData->wearableDelta->SetDesiredForce(Eigen::Vector3d(0,0,0));
    p_CommonData->wearableDelta->SetDesiredPos(Eigen::Vector3d(0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180)));

    // set flag that says haptics thread is running
    p_CommonData->hapticsThreadActive = true;

    // Set the clock that controls haptic rate
    rateClock.reset();
    rateClock.setTimeoutPeriodSeconds(0.000001);
    rateClock.start(true);

    // setup the clock that will enable display of the haptic rate
    rateDisplayClock.reset();
    rateDisplayClock.setTimeoutPeriodSeconds(1.0);
    rateDisplayClock.start(true);

    // setup the overall program time clock
    overallClock.reset();
    overallClock.start(true);

    rateDisplayCounter = 0;
    recordDataCounter = 0;
}

void haptics_thread::RecordData()
{
    recordDataCounter = 0;

    dataRecorder.time = overallClock.getCurrentTimeSeconds();
    //dataRecorder.jointAngles = p_CommonData->wearableDelta->GetJointAngles();
    dataRecorder.motorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    //dataRecorder.pos = p_CommonData->wearableDelta->GetCartesianPos();
    //dataRecorder.desiredPos = p_CommonData->wearableDelta->ReadDesiredPos();
    dataRecorder.voltageOut = p_CommonData->wearableDelta->ReadVoltageOutput();
    //dataRecorder.desiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    //dataRecorder.motorTorque = p_CommonData->wearableDelta->CalcDesiredMotorTorques(p_CommonData->wearableDelta->ReadDesiredForce());


    p_CommonData->debugData.push_back(dataRecorder);

}

