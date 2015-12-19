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
            //update Chai3D parameters
            m_tool->setLocalPos(0, 0, 0.05*sin(overallClock.getCurrentTimeSeconds())+0.02);

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
    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // Create a new world
    world = new chai3d::cWorld();

    // create a camera and insert it into the virtual world
    world->setBackgroundColor(0, 0, 0);

    // create a camera and insert it into the virtual world
    p_CommonData->p_camera = new chai3d::cCamera(world);
    world->addChild(p_CommonData->p_camera);

    // Position and orientate the camera
    p_CommonData->p_camera->set( chai3d::cVector3d (0.35/2, 0, .1),    // camera position (eye)
                                 chai3d::cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                                 chai3d::cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // X is toward camera, pos y is to right, pos z is up

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    //p_CommonData->p_camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new chai3d::cDirectionalLight(world);
    world->addChild(light);   // insert light source inside world
    light->setEnabled(true);                   // enable light source
    light->setDir(chai3d::cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam

    // create a 3D tool and add it to the world
    m_tool = new chai3d::cToolCursor(world);
    world->addChild(m_tool);
    m_tool->setRadius(0.003);
    m_tool->setLocalPos(0.0, 0.0, .05);

    // create a 3D box and add it to the world
    m_box = new chai3d::cShapeBox(.1,.1,.1);
    world->addChild(m_box);
    m_box->setLocalPos(0,0,-0.05);
    m_box->m_material->setStiffness(3.0);

    // create a haptic effect for the box so that the user can feel its surface
    newEffect = new chai3d::cEffectSurface(m_box);
    m_box->addEffect(newEffect);




    // GENERAL HAPTICS INITS=================================

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

