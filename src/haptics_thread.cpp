#include "haptics_thread.h"

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::initialize()
{
    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------
    // Create a new world
    world = new chai3d::cWorld();

    // create a camera and insert it into the virtual world
    world->setBackgroundColor(0, 0, 0);
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    p_CommonData->p_camera = new chai3d::cCamera(world);
    world->addChild(p_CommonData->p_camera);

    // Position and orientate the camera
    p_CommonData->p_camera->set( chai3d::cVector3d (0.35/2, 0, .03),    // camera position (eye)
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
    light->setDir(chai3d::cVector3d(-2.0, 0.5, -1.0));  // define the direction of the light beam

    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
    m_tool = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool); //insert the tool into the world
    toolRadius = 0.003; // set tool radius
    m_tool->setRadius(toolRadius);
    m_tool->setHapticDevice(p_CommonData->chaiDevice); // connect the haptic device to the tool
    m_tool->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool->start();
    m_tool->setShowFrame(true);
    m_tool->setFrameSize(0.05);


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------
    meshBox = new chai3d::cMesh();  // create a mesh for a box
    cCreateBox(meshBox, .1, .1, .001); // make mesh a box
    meshBox->createAABBCollisionDetector(toolRadius); // create collision detector
    world->addChild(meshBox); // add to world
    meshBox->setLocalPos(0,0,0); // set the position

    // give the box physical properties
    meshBox->m_material->setStiffness(0.03);
    meshBox->m_material->setStaticFriction(0.5);
    meshBox->m_material->setDynamicFriction(0.5);
    meshBox->m_material->setUseHapticFriction(true);

    // create a haptic effect for the box so that the user can feel its surface
    newEffect = new chai3d::cEffectSurface(m_box);
    meshBox->addEffect(newEffect);

    // create a finger object
    finger = new chai3d::cMultiMesh(); // create a virtual mesh
    world->addChild(finger); // add object to world
    finger->rotateAboutGlobalAxisDeg(chai3d::cVector3d(0,0,1), 90);
    finger->rotateAboutGlobalAxisDeg(chai3d::cVector3d(0,1,0), 90);
    finger->rotateAboutGlobalAxisDeg(chai3d::cVector3d(1,0,0), 180);
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0,0,0);
    // load an object file
    if(cLoadFileOBJ(finger, "FingerModel.obj")){
        qDebug() << "kidney file loaded";
    }
    finger->setShowEnabled(false);
    finger->setUseTransparency(false);
    finger->computeBoundaryBox(true); //compute a boundary box
    finger->setUseVertexColors(true);
    chai3d::cColorf fingerColor;
    fingerColor.setBrownSandy();
    finger->setVertexColor(fingerColor);
    finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
    finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    finger->m_material->m_specular.set(1.0, 1.0, 1.0);
    finger->setUseMaterial(true);
    finger->setHapticEnabled(false);

    double size = cSub(finger->getBoundaryMax(), finger->getBoundaryMin()).length();
    if (size > 0)
    {
        finger->scale(2.0 * m_tool->getWorkspaceRadius() / (size*20));
        qDebug() << m_tool->getWorkspaceRadius() << " " << size;
    }





    // GENERAL HAPTICS INITS=================================
    // Ensure the device is not controlling to start
    p_CommonData->posControlMode = false;
    p_CommonData->wearableDelta->SetDesiredForce(Eigen::Vector3d(0,0,0));
    p_CommonData->wearableDelta->SetDesiredPos(Eigen::Vector3d(0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180))); // kinematic neutral position

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

void haptics_thread::run()
{
    while(p_CommonData->hapticsThreadActive)
    {
        // if clock controlling haptic rate times out
        if(rateClock.timeoutOccurred())
        {
            //update Chai3D parameters
            // compute global reference frames for each object
            world->computeGlobalPositions(true);

            // update position and orientation of tool
            m_tool->updatePose();

            //Locks up running without a connected haptic device
            m_tool->computeInteractionForces();

            // get device forces
            lastComputedForce = m_tool->m_lastComputedGlobalForce;



            // stop clock while we perform haptic calcs
            rateClock.stop();

            if(p_CommonData->posControlMode == true)
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
                //RecordData();
            }


            //restart the rateClock
            rateClock.start(true);
        }        
    }

    // If we are terminating, delete the haptic device to set outputs to 0   
}






void haptics_thread::RecordData()
{
    //recordDataCounter = 0;

    //dataRecorder.time = overallClock.getCurrentTimeSeconds();
    //dataRecorder.jointAngles = p_CommonData->wearableDelta->GetJointAngles();
    //dataRecorder.motorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    //dataRecorder.pos = p_CommonData->wearableDelta->GetCartesianPos();
    //dataRecorder.desiredPos = p_CommonData->wearableDelta->ReadDesiredPos();
    //dataRecorder.voltageOut = p_CommonData->wearableDelta->ReadVoltageOutput();
    //dataRecorder.desiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    //dataRecorder.motorTorque = p_CommonData->wearableDelta->CalcDesiredMotorTorques(p_CommonData->wearableDelta->ReadDesiredForce());


    //p_CommonData->debugData.push_back(dataRecorder);

}

