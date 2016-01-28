#include "haptics_thread.h"

#define DAC_VSCALAR 819.1

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::initialize()
{
    InitAccel();
    InitGeneralChaiStuff();
    InitFingerAndTool();
    //InitPalpationEnvironment();

    // GENERAL HAPTICS INITS=================================
    // Ensure the device is not controlling to start
    p_CommonData->wearableDelta->SetDesiredForce(Eigen::Vector3d(0,0,0));
    p_CommonData->neutralPos << 0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180);
    p_CommonData->wearableDelta->SetDesiredPos(neutralPos); // kinematic neutral position

    p_CommonData->Kp = 0;
    p_CommonData->Kd = 0;

    // set flag that says haptics thread is running
    p_CommonData->hapticsThreadActive = true;

    // Set the clock that controls haptic rate
    rateClock.reset();
    rateClock.setTimeoutPeriodSeconds(0.00001);
    rateClock.start(true);

    // setup the clock that will enable display of the haptic rate
    rateDisplayClock.reset();
    rateDisplayClock.setTimeoutPeriodSeconds(1.0);
    rateDisplayClock.start(true);

    // setup the overall program time clock
    p_CommonData->overallClock.reset();
    p_CommonData->overallClock.start(true);

    //init counters to 0
    rateDisplayCounter = 0;
    recordDataCounter = 0;

    // init first contact variables
    decaySinAmpMax = 2;
    decaySinFreq = 150;
    decaySinExp = -20;
    decaySinScale = 2;
    firstTouch = true;
    decaySinAmp = 0;

    //init bandwidth variables
    p_CommonData->bandSinAmp = 0;
    p_CommonData->bandSinFreq = 0;

    // Start off not recording
    p_CommonData->recordFlag = false;
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

            accelSignal = ReadAccel();

            switch(p_CommonData->currentState)
            {
            case idle:
                UpdateVRGraphics();
                p_CommonData->wearableDelta->TurnOffControl();
                break;

            case VRControlMode:
                UpdateVRGraphics();
                ComputeVRDesiredDevicePos();
                p_CommonData->wearableDelta->PositionController(p_CommonData->Kp, p_CommonData->Kd);
                break;

            case sliderControlMode:
                UpdateVRGraphics();
                p_CommonData->wearableDelta->PositionController(p_CommonData->Kp, p_CommonData->Kd);
                break;

            case sinControlMode:
                UpdateVRGraphics();
                Eigen::Vector3d inputAxis(0,1,0);
                CommandSinPos(inputAxis);
                p_CommonData->wearableDelta->PositionController(p_CommonData->Kp, p_CommonData->Kd);
            }


            // update our rate estimate every second
            rateDisplayCounter++;
            if(rateDisplayClock.timeoutOccurred())
            {
                rateDisplayClock.stop();
                p_CommonData->hapticRateEstimate = rateDisplayCounter;
                rateDisplayCounter = 0;
                rateDisplayClock.reset();
                rateDisplayClock.start();
            }

            // record only on every 10 haptic loops
            recordDataCounter++;
            if(recordDataCounter == 10)
            {
                RecordData();
            }

            // restart rateClock
            rateClock.reset();
            rateClock.start();
        }        
    }

    // If we are terminating, delete the haptic device to set outputs to 0
    delete p_CommonData->wearableDelta;
}


void haptics_thread::UpdateVRGraphics()
{
    //update Chai3D parameters
    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    // update position and orientation of tool (and sphere that represents tool)
    m_tool0->updatePose();

    p_CommonData->chaiMagDevice0->getPosition(position0);
    p_CommonData->chaiMagDevice0->getRotation(rotation0);
    //m_curSphere0->setLocalPos(position0);
    //m_curSphere0->setLocalRot(rotation0);

    // update position of second sphere/tool
    // update position of finger to stay on proxy point
    // finger axis are not at fingerpad, so we want a translation along fingertip z axis
    chai3d::cVector3d fingerOffset(0,-0.006,0);
    fingerRotation0 = rotation0;
    fingerRotation0.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation0.rotateAboutLocalAxisDeg(1,0,0,90);
    finger->setLocalRot(fingerRotation0);
    finger->setLocalPos(m_tool0->m_hapticPoint->getGlobalPosProxy() + fingerRotation0*fingerOffset);

    //computes the interaction force for the tool proxy point
    m_tool0->computeInteractionForces();

    //use this if two tools (haptic proxies) are desired
    m_tool1->updatePose();
    p_CommonData->chaiMagDevice1->getPosition(position1);
    p_CommonData->chaiMagDevice1->getRotation(rotation1);
    //m_curSphere1->setLocalPos(position1);
    //m_curSphere1->setLocalRot(rotation1);
    m_tool1->computeInteractionForces();
}

void haptics_thread::ComputeVRDesiredDevicePos()
{
    //perform transformation to get "device forces"
    lastComputedForce0 = m_tool0->m_lastComputedGlobalForce;
    rotation0.trans();
    deviceRotation0.identity();
    deviceRotation0.rotateAboutLocalAxisDeg(0,0,1,180);
    deviceRotation0.trans();
    magTrackerLastComputedForce0 = rotation0*lastComputedForce0;
    deviceLastLastComputedForce0 = deviceLastComputedForce0;
    deviceLastComputedForce0 = deviceRotation0*rotation0*lastComputedForce0;

    //convert device "force" to a mapped position
    double forceToPosMult = 1;
    chai3d::cVector3d desiredPosMovement = forceToPosMult*deviceLastComputedForce0;
    Eigen::Vector3d neutralPos = p_CommonData->wearableDelta->neutralPos;
    Eigen::Vector3d desiredPos(3);
    desiredPos << desiredPosMovement.x()+neutralPos[0], desiredPosMovement.y()+neutralPos[1], desiredPosMovement.z()+neutralPos[2];

    // Perform position controller based on desired position
    p_CommonData->wearableDelta->SetDesiredPos(desiredPos);
}


void haptics_thread::RecordData()
{
    recordDataCounter = 0;
    dataRecorder.time = p_CommonData->overallClock.getCurrentTimeSeconds();
    dataRecorder.jointAngles = p_CommonData->wearableDelta->GetJointAngles();
    dataRecorder.motorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    dataRecorder.pos = p_CommonData->wearableDelta->GetCartesianPos();
    dataRecorder.desiredPos = p_CommonData->wearableDelta->ReadDesiredPos();
    dataRecorder.voltageOut = p_CommonData->wearableDelta->ReadVoltageOutput();
    dataRecorder.desiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    dataRecorder.motorTorque = p_CommonData->wearableDelta->CalcDesiredMotorTorques(p_CommonData->wearableDelta->ReadDesiredForce());
    dataRecorder.magTrackerPos = position0;
    dataRecorder.accelSignal = accelSignal;
    p_CommonData->debugData.push_back(dataRecorder);
}

void haptics_thread::InitGeneralChaiStuff()
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
    // X is toward camera, pos y is to right, pos z is up
    p_CommonData->p_camera->set( chai3d::cVector3d (0.18, 0.0, -0.20),//(0.25, 0, -.25),    // camera position (eye)
                                 chai3d::cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                                 chai3d::cVector3d (0.0, 0.0, -1.0));   // direction of the "up" vector

    // create a light source and attach it to the camera
    light = new chai3d::cDirectionalLight(world);
    world->addChild(light);   // insert light source inside world
    light->setEnabled(true);                   // enable light source
    light->setDir(chai3d::cVector3d(-2.0, -0.5, 1.0));  // define the direction of the light beam
}

void haptics_thread::InitFingerAndTool()
{
    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
    m_tool0 = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool0); //insert the tool into the world
    toolRadius = 0.003; // set tool radius
    m_tool0->setRadius(toolRadius);
    m_tool0->setHapticDevice(p_CommonData->chaiMagDevice0); // connect the haptic device to the tool
    m_tool0->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool0->start();

    //uncomment this if we want to use 2 tools
    m_tool1 = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool1); //insert the tool into the world
    m_tool1->setRadius(toolRadius);
    m_tool1->setHapticDevice(p_CommonData->chaiMagDevice1); // connect the haptic device to the tool
    m_tool1->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool1->start();

    // Can use this to show frames on tool if so desired
    /*//create a sphere to represent the tool
    m_curSphere0 = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere0);
    m_curSphere0->m_material->setGrayDarkSlate();
    m_curSphere0->setShowFrame(false);
    m_curSphere0->setFrameSize(0.05);

    m_curSphere1 = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere1);
    m_curSphere1->m_material->setGrayDarkSlate();
    m_curSphere1->setShowFrame(false);
    m_curSphere1->setFrameSize(0.05);*/


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------
    /*
    // create a box and give it physical properties
    meshBox = new chai3d::cMesh();  // create a mesh for a box
    cCreateBox(meshBox, .07, .07, .035); // make mesh a box
    meshBox->createAABBCollisionDetector(toolRadius); // create collision detector
    world->addChild(meshBox); // add to world
    meshBox->setLocalPos(0,0,.02); // set the position

    // give the box physical properties
    meshBox->m_material->setStiffness(200);
    meshBox->m_material->setStaticFriction(0.7);
    meshBox->m_material->setDynamicFriction(0.7);
    meshBox->m_material->setUseHapticFriction(true);*/

    // create a finger object
    finger = new chai3d::cMultiMesh(); // create a virtual mesh
    world->addChild(finger); // add object to world
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0,0,0);

    // load an object file
    if(cLoadFileOBJ(finger, "./Resources/FingerModel.obj")){
        qDebug() << "finger file loaded";
    }

    // set params
    finger->setShowEnabled(true);
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
        finger->scale(1.0);
        qDebug() << m_tool0->getWorkspaceRadius() << " " << size;
    }
}

void haptics_thread::InitPalpationEnvironment()
{
    /*
    // create a virtual mesh
    p_table = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_table);

    // load object
    //cLoadFileOBJ(p_petriDish, "./Resources/table/table.obj");
    bool fileLoad = p_table->loadFromFile("./Resources/table/table.obj");

    // compute a boundary box
    p_table->computeBoundaryBox(true);

    // get dimensions of object
    double size = cSub(p_table->getBoundaryMax(), p_table->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_table->scale(1);
    }

    // compute collision detection algorithm
    p_table->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_table->setStiffness(100, true);*/

    double size;


    //----------------------------------------------Create Petri Dish---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_petriDish = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_petriDish);

    //load the object from file
    //cLoadFileOBJ(p_CommonData->p_petriDish, "./Resources/petri_dish/petri_dish.obj");
    p_CommonData->p_petriDish->loadFromFile("./Resources/petri_dish/petri_dish.obj");

    p_CommonData->p_petriDish->rotateAboutLocalAxisDeg(1,0,0,180);

    // compute a boundary box
    p_CommonData->p_petriDish->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_petriDish->getBoundaryMax(), p_CommonData->p_petriDish->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_petriDish->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_petriDish->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_petriDish->setStiffness(100, true);

    p_CommonData->p_petriDish->setTransparencyLevel(0.4, true, true);

    p_CommonData->p_petriDish->setUseVertexColors(true);
    chai3d::cColorf petriDishColor;
    petriDishColor.setGrayDark();
    p_CommonData->p_petriDish->setVertexColor(petriDishColor);
    p_CommonData->p_petriDish->m_material->m_ambient.set(0.1, 0.1, 0.1);
    p_CommonData->p_petriDish->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    p_CommonData->p_petriDish->m_material->m_specular.set(1.0, 1.0, 1.0);
    p_CommonData->p_petriDish->setUseMaterial(true);

    //----------------------------------------------Create Tissue One---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueOne = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueOne);

    p_CommonData->p_tissueOne->setLocalPos(0,0,-.005);

    //load the object from file
    //cLoadFileOBJ(p_CommonData->p_tissueOne, "./Resources/tissue_1/tissue_1.obj");
    p_CommonData->p_tissueOne->loadFromFile("./Resources/tissue_1/tissue_1.obj");

    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(1,0,0,180);

    // compute a boundary box
    p_CommonData->p_tissueOne->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueOne->getBoundaryMax(), p_CommonData->p_tissueOne->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueOne->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueOne->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueOne->setTransparencyLevel(0.4, true, true);
    p_CommonData->p_tissueOne->setStiffness(STIFFNESS_BASELINE, true);
    p_CommonData->p_tissueOne->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);

    //----------------------------------------------Create Tissue Two---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueTwo = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueTwo);    

    p_CommonData->p_tissueTwo->setLocalPos(0,0,-.025);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueTwo->loadFromFile("./Resources/tissue_2/tissue_2.obj");



    // compute a boundary box
    p_CommonData->p_tissueTwo->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueTwo->getBoundaryMax(), p_CommonData->p_tissueTwo->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueTwo->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueTwo->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueTwo->setStiffness(STIFFNESS_BASELINE + STIFFNESS_INCREMENT, true);
    p_CommonData->p_tissueTwo->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);
    //----------------------------------------------Create Tissue Three---------------------------------------------------

    // create a virtual mesh
    p_CommonData->p_tissueThree = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueThree);

    p_CommonData->p_tissueThree->setLocalPos(0,0,-.025);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueThree->loadFromFile("./Resources/tissue_3/tissue_3.obj");

    // compute a boundary box
    p_CommonData->p_tissueThree->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueThree->getBoundaryMax(), p_CommonData->p_tissueThree->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueThree->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueThree->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueThree->setStiffness(STIFFNESS_BASELINE + 2* STIFFNESS_INCREMENT, true);
    p_CommonData->p_tissueThree->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);

    //----------------------------------------------Create Tissue Four---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueFour = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueFour);

    p_CommonData->p_tissueFour->setLocalPos(0,0,-.025);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueFour->loadFromFile("./Resources/tissue_4/tissue_4.obj");

    // compute a boundary box
    p_CommonData->p_tissueFour->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueFour->getBoundaryMax(), p_CommonData->p_tissueFour->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueFour->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueFour->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueFour->setStiffness(STIFFNESS_BASELINE + 3*STIFFNESS_INCREMENT, true);
    p_CommonData->p_tissueFour->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);

    //----------------------------------------------Create Tissue Five---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueFive = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueFive);

    p_CommonData->p_tissueFive->setLocalPos(0,0,-.025);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueFive->loadFromFile("./Resources/tissue_5/tissue_5.obj");

    // compute a boundary box
    p_CommonData->p_tissueFive->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueFive->getBoundaryMax(), p_CommonData->p_tissueFive->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueFive->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueFive->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueFive->setStiffness(STIFFNESS_BASELINE + 4*STIFFNESS_INCREMENT, true);
    p_CommonData->p_tissueFive->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);

    //----------------------------------------------Create Tissue Six---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueSix = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueSix);

    p_CommonData->p_tissueSix->setLocalPos(0,0,-.025);
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueSix->loadFromFile("./Resources/tissue_6/tissue_6.obj");

    // compute a boundary box
    p_CommonData->p_tissueSix->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueSix->getBoundaryMax(), p_CommonData->p_tissueSix->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueSix->scale(1);
    }

    // compute collision detection algorithm
    p_CommonData->p_tissueSix->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueSix->setStiffness(STIFFNESS_BASELINE + 5*STIFFNESS_INCREMENT, true);
    p_CommonData->p_tissueSix->setFriction(STATIC_FRICTION, DYNAMIC_FRICTION, TRUE);

    //----------------------------------------------Create Tissue Seven---------------------------------------------------
    // create a virtual mesh
    p_CommonData->p_tissueSeven = new chai3d::cMultiMesh();

    // add object to world
    world->addChild(p_CommonData->p_tissueSeven);

    p_CommonData->p_tissueSeven->setLocalPos(0,0,-.02);
    p_CommonData->p_tissueSeven->rotateAboutLocalAxisDeg(1,0,0,180);

    //load the object from file
    p_CommonData->p_tissueSeven->loadFromFile("./Resources/tissue_7/tissue_7.obj");

    // compute a boundary box
    p_CommonData->p_tissueSeven->computeBoundaryBox(true);

    // get dimensions of object
    size = cSub(p_CommonData->p_tissueSeven->getBoundaryMax(), p_CommonData->p_tissueSeven->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        p_CommonData->p_tissueSeven->scale(1);
    }

    p_CommonData->p_tissueSeven->setShowEnabled(false);
    //------------------------------------------------------------------------------------------------------
    //Set initial transparency of tissue
    p_CommonData->m_flagTissueTransparent = false;
}

void haptics_thread::CommandSinPos(Eigen::Vector3d inputMotionAxis)
{
    //perform check on validity of axis
    double inputAxisMag = inputMotionAxis.norm();
    if ((inputAxisMag < 1.01) & (inputAxisMag > 0.99))
    {
        // ----------------- START SINUSOID -----------------------------------
        double currTime = p_CommonData->overallClock.getCurrentTimeSeconds() - p_CommonData->sinStartTime;
        double rampTime = 2.0;
        // scale to avoid abrupt starting input
        double scaledBandSinAmp = currTime/rampTime*p_CommonData->bandSinAmp;
        if (scaledBandSinAmp > p_CommonData->bandSinAmp)
        {
            scaledBandSinAmp = p_CommonData->bandSinAmp;
        }
        Eigen::Vector3d sinPos = (scaledBandSinAmp*sin(2*PI*p_CommonData->bandSinFreq*currTime))*inputMotionAxis + p_CommonData->neutralPos;
        p_CommonData->wearableDelta->SetDesiredPos(sinPos);

        // If time is greater than 10 seconds, pause for a bit
        if (currTime > 10)
        {
            p_CommonData->wearableDelta->SetDesiredPos(p_CommonData->neutralPos);
        }

        // If time is greater than 12 seconds, set ourselves up to go again
        if (currTime > 12)
        {
            p_CommonData->sinStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
            p_CommonData->bandSinFreq = p_CommonData->bandSinFreq + 0.2;
            p_CommonData->bandSinFreqDisp = p_CommonData->bandSinFreq;
            if (p_CommonData->bandSinFreq > 9.9)
            {
                p_CommonData->currentState = idle;
            }
        }
    }
    else
    {
        qDebug("invalid motion axis");
    }
}

void haptics_thread::InitAccel()
{
#ifdef SENSORAY626
    // Sets the 3.3 V output for running the accelerometer
    double VoltOut = 3.3;
    long writeData = (long)(VoltOut*DAC_VSCALAR);

    S626_WriteDAC(0,3,writeData);
    qDebug() << "made it";
    qDebug() << writeData;

    // PERFORM INITIALIZATION OF SENSORAY FOR READING IN ADC
    // Allocate data structures. We allocate enough space for maximum possible
    // number of items (16) even though this example only has 3 items. Although
    // this is not required, it is the recommended practice to prevent programming
    // errors if your application ever modifies the number of items in the poll list.

    // Populate the poll list.
    poll_list[0] = 0 | RANGE_5V | EOPL; // Chan 0, ±5V range, mark as list end.
    // Prepare for A/D conversions by passing the poll list to the driver.
    S626_ResetADC( 0, poll_list );
    // Digitize all items in the poll list. As long as the poll list is not modified,
    // S626_ReadADC() can be called repeatedly without calling S626_ResetADC() again.
    // This could be implemented as two calls: S626_StartADC() and S626_WaitDoneADC().
    S626_ReadADC( 0, databuf ); //board 0, data in databuf
#endif
}

chai3d::cVector3d haptics_thread::ReadAccel()
{
#ifdef SENSORAY626
    S626_ReadADC(0, databuf);
    chai3d::cVector3d returnVec(databuf[0], databuf[1], databuf[2]);
    return returnVec;
#endif SENSORAY626

    chai3d::cVector3d emptyVec;
    return emptyVec; //if sensoray is not active, just return 0
}



double haptics_thread::ComputeContactVibration(){
    //create a contact vibration that depends on position or velocity
    // if we are registering a contact
    if (abs(deviceLastComputedForce0.z()) > 0.00001)
    {
        // if this is the first time in contact
        if (firstTouch)
        {
            // start the time ticking
            decaySinTime = 0;
            decaySinTime += 0.001;

            // get the sinusoid amplitude based last velocity
            p_CommonData->chaiMagDevice0->getLinearVelocity(estimatedVel0);
            this->decaySinAmp = this->decaySinScale*estimatedVel0.z();

            //check if amplitude exceeds desired amount
            if (decaySinAmp > decaySinAmpMax)
            {
                decaySinAmp = decaySinAmpMax;
            }

            // calculate contributed force
            computedPosAdd = -pow(E_VALUE, decaySinExp*decaySinTime) * decaySinAmp * sin(decaySinFreq*2*PI*decaySinTime);
            qDebug() << computedPosAdd;

            // set next force to not be "first touch"
            firstTouch = false;
        }
        else
        {
            //still inside wall with "contact"
            computedPosAdd = -pow(E_VALUE, decaySinExp*decaySinTime) * decaySinAmp * sin(decaySinFreq*2*PI*decaySinTime);

            //increment decay time counter
            decaySinTime += 0.001;
        }
    }
    else
    {
        if (!firstTouch)
        {
            // just came out of contact, reset that next one will be "first touch"
            firstTouch = true;

            // reset the decay time, amplitude, and added "pos"
            decaySinTime = 0;
            decaySinAmp = 0;
            computedPosAdd = 0;
        }
    }
    return computedPosAdd;
}

void haptics_thread::InitDynamicBodies()
{

    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------
/*
    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);
    world->addChild(ODEWorld);

    //give world gravity
    ODEWorld->setGravity(chai3d::cVector3d(0.0, 0.0, 9.81));
    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);

    // Create an ODE Block
    double boxSize = 0.05;
    ODEBody0 = new cODEGenericBody(ODEWorld);
    // create a dynamic model of the ODE object
    ODEBody0->createDynamicBox(boxSize, boxSize, boxSize);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    meshBox = new chai3d::cMesh();
    cCreateBox(meshBox, boxSize, boxSize, boxSize); // make mesh a box
    meshBox->createAABBCollisionDetector(toolRadius);
    chai3d::cMaterial mat0;
    mat0.setBlueRoyal();
    mat0.setStiffness(300);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    meshBox->setMaterial(mat0);

    // add mesh to ODE object
    ODEBody0->setImageModel(meshBox);

    // set mass of box
    ODEBody0->setMass(0.5);

    // set position of box
    ODEBody0->setLocalPos(0.0,0,0);

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0.05), chai3d::cVector3d(0.0, 0.0 ,-1.0));

    //create ground
    ground = new chai3d::cMesh();
    world->addChild(ground);

    //create a plane
    double groundSize = 5;
    chai3d::cCreatePlane(ground, groundSize, groundSize);

    //position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0,0,0.05);

    //define some material properties
    chai3d::cMaterial matGround;
    matGround.setStiffness(300);
    matGround.setDynamicFriction(0.4);
    matGround.setStaticFriction(0.0);
    matGround.setWhite();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);*/
}

void haptics_thread::SimulateDynamicBodies()
{
    double timeInterval = rateClock.getCurrentTimeSeconds();

    //---------------------------------------------------
    // Implement Dynamic simulation
    //---------------------------------------------------
    int numInteractionPoints = m_tool0->getNumInteractionPoints();
    for (int i=0; i<numInteractionPoints; i++)
    {
        // get pointer to next interaction point of tool
        chai3d::cHapticPoint* interactionPoint = m_tool0->getInteractionPoint(i);

        // check primary contact point if available
        if (interactionPoint->getNumCollisionEvents() > 0)
        {
            chai3d::cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

            // given the mesh object we may be touching, we search for its owner which
            // could be the mesh itself or a multi-mesh object. Once the owner found, we
            // look for the parent that will point to the ODE object itself.
            chai3d::cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

            // cast to ODE object
            cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

            // if ODE object, we apply interaction forces
            if (ODEobject != NULL)
            {
                ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                                                   collisionEvent->m_globalPos);
            }
        }
    }
    numInteractionPoints = m_tool1->getNumInteractionPoints();
    for (int i=0; i<numInteractionPoints; i++)
    {
        // get pointer to next interaction point of tool
        chai3d::cHapticPoint* interactionPoint = m_tool1->getInteractionPoint(i);

        // check primary contact point if available
        if (interactionPoint->getNumCollisionEvents() > 0)
        {
            chai3d::cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

            // given the mesh object we may be touching, we search for its owner which
            // could be the mesh itself or a multi-mesh object. Once the owner found, we
            // look for the parent that will point to the ODE object itself.
            chai3d::cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

            // cast to ODE object
            cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

            // if ODE object, we apply interaction forces
            if (ODEobject != NULL)
            {
                ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                                                   collisionEvent->m_globalPos);
            }
        }
    }

    ODEWorld->updateDynamics(timeInterval/5);
}


