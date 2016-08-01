#include "haptics_thread.h"

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
    InitEnvironments();
    InitDynamicBodies();

    // GENERAL HAPTICS INITS=================================
    // Ensure the device is not controlling to start
    p_CommonData->wearableDelta0->TurnOffControl();
    p_CommonData->wearableDelta1->TurnOffControl();

    p_CommonData->wearableDelta0->SetDesiredPos(p_CommonData->wearableDelta0->neutralPos); // kinematic neutral position
    p_CommonData->wearableDelta1->SetDesiredPos(p_CommonData->wearableDelta1->neutralPos); // kinematic neutral position

    // set flag that says haptics thread is running
    p_CommonData->hapticsThreadActive = true;
    p_CommonData->environmentChange = false;

    // set to provide feedback when running VR control mode
    p_CommonData->tactileFeedback = true;

    // set up palpation post trial clock
    p_CommonData->palpPostTrialClock.reset();
    p_CommonData->palpPostTrialClock.setTimeoutPeriodSeconds(2.0);

    // Set the clock that controls haptic rate
    rateClock.reset();
    rateClock.setTimeoutPeriodSeconds(0.000001);
    rateClock.start(true);

    // setup the clock that will enable display of the haptic rate
    rateDisplayClock.reset();
    rateDisplayClock.setTimeoutPeriodSeconds(1.0);
    rateDisplayClock.start(true);

    // setup the overall program time clock
    p_CommonData->overallClock.reset();
    p_CommonData->overallClock.start(true);

    // setup the calibration clock
    p_CommonData->calibClock.reset();

    // init values for first time through on filter
    lastFilteredDeviceForce0.set(0,0,0);
    lastFilteredDeviceForce1.set(0,0,0);

    currTime = 0;
    lastTime = 0;

    //init counters to 0
    rateDisplayCounter = 0;
    recordDataCounter = 0;

    //init bandwidth variables
    p_CommonData->bandSinAmp = 0;
    p_CommonData->bandSinFreq = 0;

    // Start off not recording
    p_CommonData->recordFlag = false;

    p_CommonData->currentControlState = idleControl;
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
            Eigen::Vector3d inputAxis(0,0,1); // input axis for sin control and circ control modes
            switch(p_CommonData->currentControlState)
            {
            case initCalibControl:
                UpdateVRGraphics();
                SetInitJointAngles();
                p_CommonData->wearableDelta0->IndivJointController(p_CommonData->desJointInits0, p_CommonData->jointKp, p_CommonData->jointKd);
                p_CommonData->wearableDelta1->IndivJointController(p_CommonData->desJointInits1, p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case idleControl:
                UpdateVRGraphics();
                p_CommonData->wearableDelta0->TurnOffControl();
                p_CommonData->wearableDelta1->TurnOffControl();
                break;

            case VRControlMode:
                UpdateVRGraphics();
                ComputeVRDesiredDevicePos();
                p_CommonData->wearableDelta0->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                p_CommonData->wearableDelta1->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case sliderControlMode:
                UpdateVRGraphics();
                p_CommonData->wearableDelta0->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                p_CommonData->wearableDelta1->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case sinControlMode:
                UpdateVRGraphics();                
                CommandSinPos(inputAxis);
                p_CommonData->wearableDelta0->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case circControlMode:
                UpdateVRGraphics();
                CommandCircPos(inputAxis);
                p_CommonData->wearableDelta0->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;
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
            if(recordDataCounter == 5)
            {
                recordDataCounter = 0;
                if(p_CommonData->recordFlag == true)
                {
                    RecordData();
                }
            }

            // restart rateClock
            rateClock.reset();
            rateClock.start();
        }        
    }

    // If we are terminating, delete the haptic device to set outputs to 0
    delete p_CommonData->wearableDelta0;
    delete p_CommonData->wearableDelta1;
}

void haptics_thread::UpdateVRGraphics()
{
    // Update camera Pos
    double xPos = p_CommonData->camRadius*cos(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double yPos = p_CommonData->camRadius*sin(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double zPos = p_CommonData->camRadius*cos(p_CommonData->polar*PI/180.0);
    p_CommonData->cameraPos.set(xPos, yPos, zPos);

#ifdef OCULUS
    p_CommonData->lookatPos.set(-1, 0, p_CommonData->cameraPos.z());
#endif

#ifndef OCULUS
    p_CommonData->lookatPos.set(0, 0, 0);
#endif
    // update camera parameters
    p_CommonData->p_camera->set( p_CommonData->cameraPos,
                                 p_CommonData->lookatPos,
                                 p_CommonData->upVector);

    if(p_CommonData->environmentChange == true)
    {
        p_CommonData->environmentChange = false;
        switch(p_CommonData->currentEnvironmentState)
        {
        case none:
            break;

        case experimentFriction:
            world->clearAllChildren();
            RenderExpFriction();
            break;

        case friction:
            world->clearAllChildren();
            RenderTwoFriction();
            break;

        case palpation:
            world->clearAllChildren();
            RenderPalpation();
            break;

        case experimentPalpationLine:
            world->clearAllChildren();
            RenderPalpation();
            break;

        case dynamicBodies:
            world->clearAllChildren();
            RenderDynamicBodies();
            break;
        }
    }

    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    m_tool0->updateFromDevice();  // update position and orientation of tool 0(and sphere that represents tool)
    p_CommonData->chaiMagDevice0->getPosition(position0); // get position and rotation of the delta mechanism (already transformed from magTracker)
    p_CommonData->chaiMagDevice0->getRotation(rotation0);
    deviceRotation0 = rotation0; //the chai device already rotates the tracker return based on the finger harness
    m_curSphere0->setLocalPos(position0); // set the visual representation to match
    m_curSphere0->setLocalRot(rotation0);
    m_tool0->computeInteractionForces();

    // update position of finger to stay on proxy point    
    chai3d::cVector3d fingerOffset(0,-0.006,0); // finger axis are not at fingerpad, so we want a translation outward on fingertip
    fingerRotation0 = rotation0;
    fingerRotation0.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation0.rotateAboutLocalAxisDeg(1,0,0,90);
    finger->setLocalRot(fingerRotation0);
    finger->setLocalPos(m_tool0->m_hapticPoint->getGlobalPosProxy() + fingerRotation0*fingerOffset);

    // update position and orientation of tool 1
    m_tool1->updateFromDevice();
    p_CommonData->chaiMagDevice1->getPosition(position1);
    p_CommonData->chaiMagDevice1->getRotation(rotation1);
    deviceRotation1 = rotation1;
    m_curSphere1->setLocalPos(position1);
    m_curSphere1->setLocalRot(rotation1);
    m_tool1->computeInteractionForces();

    // update position of finger to stay on proxy point    
    chai3d::cVector3d thumbOffset(0,-0.01,0); // finger axis are not at fingerpad, so we want a translation outward on fingertip
    fingerRotation1 = rotation1;
    fingerRotation1.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation1.rotateAboutLocalAxisDeg(1,0,0,90);
    thumb->setLocalRot(fingerRotation1);
    thumb->setLocalPos(m_tool1->m_hapticPoint->getGlobalPosProxy() + fingerRotation1*thumbOffset);

    // perform our dynamic body updates if we are in a dynamic environment
    if(p_CommonData->currentEnvironmentState == dynamicBodies)
    {
        currTime = p_CommonData->overallClock.getCurrentTimeSeconds();
        double timeInterval = currTime - lastTime;
        if(timeInterval > 0.01) timeInterval = 0.01;
        //---------------------------------------------------
        // Implement Dynamic simulation
        //---------------------------------------------------
        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        int numInteractionPoints = m_tool0->getNumHapticPoints();
        for (int j=0; j<numInteractionPoints; j++)
        {
            // get pointer to next interaction point of tool
            chai3d::cHapticPoint* interactionPoint = m_tool0->getHapticPoint(j);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();
            for (int k=0; k<numContacts; k++)
            {
                chai3d::cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(k);

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
        numInteractionPoints = m_tool1->getNumHapticPoints();
        for (int j=0; j<numInteractionPoints; j++)
        {
            // get pointer to next interaction point of tool
            chai3d::cHapticPoint* interactionPoint = m_tool1->getHapticPoint(j);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();
            for (int k=0; k<numContacts; k++)
            {
                chai3d::cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(k);

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
        // update simulation
        ODEWorld->updateDynamics(timeInterval);
        lastTime = currTime;
    }

    /*
    // THIS SLOPPY STUFF TO HANDLE THE PALPATION VISIBILITY AFTER TRIALS
    // mainwindow makes the line visible after palpationLine trial, this makes it opaque again and starts the next trial
    if(p_CommonData->palpPostTrialClock.timeoutOccurred())
    {
        p_CommonData->palpPostTrialClock.stop();
        p_CommonData->palpPostTrialClock.reset();

        // increment trial no
        p_CommonData->trialNo = p_CommonData->trialNo + 1;

        QString type = (p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "type", NULL ));

        qDebug() << type;

        if (type == "training")
        {
            p_CommonData->currentExperimentState = palpationLineTrial;
            // rotate line to new location
            double lastRotation = p_CommonData->tissueRot;
            p_CommonData->tissueRot = atoi(p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Angles", NULL ));

            rotateTissueLine(-lastRotation);
            rotateTissueLine(p_CommonData->tissueRot);
            p_CommonData->p_indicator->setTransparencyLevel(0, true);
            p_CommonData->recordFlag = true;
        }

        else if(type == "break")
        {
            p_CommonData->currentExperimentState = palpationLineBreak;
            p_CommonData->p_tissueOne->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSeven->setTransparencyLevel(1, true);
            p_CommonData->p_tissueEight->setTransparencyLevel(1, true);
            p_CommonData->p_tissueNine->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTen->setTransparencyLevel(1, true);
            p_CommonData->p_tissueEleven->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwelve->setTransparencyLevel(1, true);
            p_CommonData->p_indicator->setTransparencyLevel(0, true);
        }

        else if(type == "trial")
        {
            p_CommonData->currentExperimentState = palpationLineTrial;
            // rotate line to new location
            double lastRotation = p_CommonData->tissueRot;
            p_CommonData->tissueRot = atoi(p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Angles", NULL ));

            rotateTissueLine(-lastRotation);
            rotateTissueLine(p_CommonData->tissueRot);

            p_CommonData->p_tissueOne->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSeven->setTransparencyLevel(1, true);
            p_CommonData->p_tissueEight->setTransparencyLevel(1, true);
            p_CommonData->p_tissueNine->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTen->setTransparencyLevel(1, true);
            p_CommonData->p_tissueEleven->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwelve->setTransparencyLevel(1, true);
            p_CommonData->p_indicator->setTransparencyLevel(0, true);
        }

        else if (type == "end")
        {
            p_CommonData->currentExperimentState = endExperiment;
        }

        // rotate tissue line indicator back to 0
        double lastDispRotation = p_CommonData->indicatorRot;
        rotateTissueLineDisp(-lastDispRotation);
    }*/
}

void haptics_thread::ComputeVRDesiredDevicePos()
{
    //perform transformation to get "device forces"
    computedForce0 = m_tool0->getDeviceGlobalForce();
    computedForce1 = m_tool1->getDeviceGlobalForce();

    // rotation of delta mechanism in world frame (originally from mag tracker, but already rotated the small bend angle of finger)
    rotation0.trans();
    rotation1.trans();

    // create another rotation (this essentially just flips the tail in/out of mag tracker
    deviceRotation0.identity();
    deviceRotation0.rotateAboutLocalAxisDeg(0,0,1,180);
    deviceRotation0.trans();
    deviceRotation1.identity();
    deviceRotation1.rotateAboutLocalAxisDeg(0,0,1,180);
    deviceRotation1.trans();

    // this are the forces in the device frames
    deviceComputedForce0 = deviceRotation0*rotation0*computedForce0; // rotation between force in world and delta frames
    deviceComputedForce1 = deviceRotation1*rotation1*computedForce1; // rotation between force in world and delta frames

    // write down the most recent device and world forces for recording
    deviceForceRecord0 << deviceComputedForce0.x(),deviceComputedForce0.y(),deviceComputedForce0.z();
    globalForceRecord0 << computedForce0.x(), computedForce0.y(), computedForce0.z();
    deviceForceRecord1 << deviceComputedForce1.x(),deviceComputedForce1.y(),deviceComputedForce1.z();
    globalForceRecord1 << computedForce1.x(), computedForce1.y(), computedForce1.z();

    // filter param
    double alpha = 0.5;

    // get filtered force
    filteredDeviceForce0 = alpha*deviceComputedForce0 + (1-alpha)*lastFilteredDeviceForce0;

    //convert device "force" to a mapped position
    double forceToPosMult = 1.0/1.588; // based on lateral stiffness of finger (averaged directions from Gleeson paper) (1.588 N/mm)    

    // Pos movements in delta mechanism frame (index)
    chai3d::cVector3d desiredPosMovement0 = forceToPosMult*filteredDeviceForce0; //this is only for lateral if we override normal later
    chai3d::cVector3d desiredPosMovement1 = forceToPosMult*filteredDeviceForce1; //this is only for lateral if we override normal later

    // check to see if we are rendering only normal or only lateral
    if(p_CommonData->flagNormal == false)
    {
        desiredPosMovement0.z(0);
        desiredPosMovement1.z(0);
    }

    if(p_CommonData->flagLateral == false)
    {
        desiredPosMovement0.x(0);
        desiredPosMovement0.y(0);
        desiredPosMovement1.x(0);
        desiredPosMovement1.y(0);
    }    

    // don't allow the tactor to move away from finger
    double vertPosMovement0 = desiredPosMovement0.z();
    double vertPosMovement1 = desiredPosMovement1.z();
    if(vertPosMovement0 > 0)
        vertPosMovement0 = 0;
    if(vertPosMovement1 > 0)
        vertPosMovement1 = 0;

    Eigen::Vector3d neutralPos0 = p_CommonData->wearableDelta0->neutralPos;
    Eigen::Vector3d desiredPos0(3);
    desiredPos0 << desiredPosMovement0.x()+neutralPos0[0], desiredPosMovement0.y()+neutralPos0[1], vertPosMovement0+neutralPos0[2];
    Eigen::Vector3d neutralPos1 = p_CommonData->wearableDelta1->neutralPos;
    Eigen::Vector3d desiredPos1(3);
    desiredPos1 << desiredPosMovement1.x()+neutralPos1[0], desiredPosMovement1.y()+neutralPos1[1], vertPosMovement1+neutralPos1[2];

    // if the experimental condition is no feedback, tell it to move to neutral pos
    if(p_CommonData->tactileFeedback == 0)
    {
        desiredPos0 << neutralPos0[0], neutralPos0[1], neutralPos0[2];
        desiredPos1 << neutralPos1[0], neutralPos1[1], neutralPos1[2];
    }

    // Perform control based on desired position
    p_CommonData->wearableDelta0->SetDesiredPos(desiredPos0);
    p_CommonData->wearableDelta1->SetDesiredPos(desiredPos1);

    lastFilteredDeviceForce0 = filteredDeviceForce0;
    lastFilteredDeviceForce1 = filteredDeviceForce1;
}

void haptics_thread::RecordData()
{
    recordDataCounter = 0;
    dataRecorder.time = p_CommonData->overallClock.getCurrentTimeSeconds();
    dataRecorder.jointAngles = p_CommonData->wearableDelta0->GetJointAngles();
    dataRecorder.motorAngles = p_CommonData->wearableDelta0->GetMotorAngles();
    dataRecorder.pos = p_CommonData->wearableDelta0->GetCartesianPos();
    dataRecorder.desiredPos = p_CommonData->wearableDelta0->ReadDesiredPos();
    dataRecorder.voltageOut = p_CommonData->wearableDelta0->ReadVoltageOutput();
    dataRecorder.VRInteractionForce = deviceForceRecord0; // last force on tool0
    dataRecorder.VRInteractionForceGlobal = globalForceRecord0; // last force on tool0 in global coords
    dataRecorder.motorTorque = p_CommonData->wearableDelta0->motorTorques;
    dataRecorder.magTrackerPos0 = position0;
    dataRecorder.magTrackerPos1 = position1;
    dataRecorder.accelSignal = accelSignal;
    dataRecorder.tactileFeedback = p_CommonData->tactileFeedback;
    dataRecorder.referenceFirst = p_CommonData->referenceFirst;
    dataRecorder.pairNo = p_CommonData->pairNo;
    dataRecorder.referenceFriction = p_CommonData->referenceFriction;
    dataRecorder.comparisonFriction = p_CommonData->comparisonFriction;
    dataRecorder.subjectAnswer = p_CommonData->subjectAnswer;
    dataRecorder.lumpLocation = p_CommonData->p_tissueLump->getLocalPos();
    dataRecorder.lineAngle = p_CommonData->indicatorRot;
    dataRecorder.lineAngleTruth = p_CommonData->tissueRot;
    dataRecorder.deviceRotation = deviceRotation;

    p_CommonData->debugData.push_back(dataRecorder);
}

void haptics_thread::InitGeneralChaiStuff()
{
    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------
    // Create a new world
    world = new chai3d::cWorld();
    p_CommonData->p_world = world;

    // create a camera and insert it into the virtual world
    world->setBackgroundColor(0, 0, 0);
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    p_CommonData->p_camera = new chai3d::cCamera(world);
    world->addChild(p_CommonData->p_camera);

    // Position and orientate the camera
    // X is toward camera, pos y is to right, pos z is up
    p_CommonData->cameraPos.set(0.18, 0.0, 0);
    p_CommonData->lookatPos.set(0.0, 0.0, 0.0);
    p_CommonData->upVector.set(0.0, 0.0, -1.0);
    p_CommonData->p_camera->set( p_CommonData->cameraPos,//(0.25, 0, -.25),    // camera position (eye)
                                 p_CommonData->lookatPos,    // lookat position (target)
                                 p_CommonData->upVector);   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    p_CommonData->p_camera->setClippingPlanes(0.01, 20.0);

    p_CommonData->azimuth = 0.0;
    p_CommonData->polar = 150.0;
    p_CommonData->camRadius = 0.25;

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
    toolRadius = 0.002; // set tool radius
    m_tool0->setRadius(toolRadius);
    m_tool0->setHapticDevice(p_CommonData->chaiMagDevice0); // connect the haptic device to the tool
    m_tool0->setShowContactPoints(true, false, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool0->enableDynamicObjects(true);
    m_tool0->start();

    //uncomment this if we want to use 2 tools
    m_tool1 = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool1); //insert the tool into the world
    m_tool1->setRadius(toolRadius);
    m_tool1->setHapticDevice(p_CommonData->chaiMagDevice1); // connect the haptic device to the tool
    m_tool1->setShowContactPoints(true, false, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool1->enableDynamicObjects(true);
    m_tool1->start();

    // Can use this to show frames on tool if so desired
    //create a sphere to represent the tool
    m_curSphere0 = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere0);
    m_curSphere0->m_material->setGrayDarkSlate();
    m_curSphere0->setShowFrame(true);
    m_curSphere0->setFrameSize(0.05);

    m_curSphere1 = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere1);
    m_curSphere1->m_material->setBlueAqua();
    m_curSphere1->setShowFrame(true);
    m_curSphere1->setFrameSize(0.05);


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------
    // create a finger object
    finger = new chai3d::cMultiMesh(); // create a virtual mesh
    world->addChild(finger); // add object to world
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0.0, 0.0, 0.0);

    thumb = new chai3d::cMultiMesh(); //create the thumb
    world->addChild(thumb);
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0,0,0);

    // load an object file
    if(cLoadFileOBJ(finger, "./Resources/FingerModel.obj")){
        qDebug() << "finger file loaded";
    }
    if(cLoadFileOBJ(thumb, "./Resources/FingerModelThumb.obj")){
        qDebug() << "thumb file loaded";
    }

    // set params for finger
    finger->setShowEnabled(true);
    finger->setUseVertexColors(true);
    chai3d::cColorf fingerColor;
    fingerColor.setBrownSandy();
    finger->setVertexColor(fingerColor);
    finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
    finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    finger->m_material->m_specular.set(1.0, 1.0, 1.0);
    finger->setUseMaterial(true);
    finger->setHapticEnabled(false);

    // set params for thumb
    thumb->setShowEnabled(true);
    thumb->setUseVertexColors(true);
    chai3d::cColorf thumbColor;
    thumbColor.setBrownSandy();
    thumb->setVertexColor(thumbColor);
    thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
    thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
    thumb->setUseMaterial(true);
    thumb->setHapticEnabled(false);
}

void haptics_thread::InitEnvironments()
{
    p_CommonData->p_frictionBox1 = new chai3d::cMesh();
    p_CommonData->p_frictionBox2 = new chai3d::cMesh();

    p_CommonData->p_tissueOne = new chai3d::cMultiMesh();
    p_CommonData->p_tissueTwo = new chai3d::cMultiMesh();
    p_CommonData->p_tissueThree = new chai3d::cMultiMesh();
    p_CommonData->p_tissueFour = new chai3d::cMultiMesh();
    p_CommonData->p_tissueFive = new chai3d::cMultiMesh();
    p_CommonData->p_tissueSix = new chai3d::cMultiMesh();
    p_CommonData->p_tissueSeven = new chai3d::cMultiMesh();
    p_CommonData->p_tissueEight = new chai3d::cMultiMesh();
    p_CommonData->p_tissueNine = new chai3d::cMultiMesh();
    p_CommonData->p_tissueTen = new chai3d::cMultiMesh();
    p_CommonData->p_tissueEleven = new chai3d::cMultiMesh();
    p_CommonData->p_tissueTwelve = new chai3d::cMultiMesh();
    p_CommonData->p_indicator = new chai3d::cMultiMesh();
    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueSeven->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueEight->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueNine->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueTen->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueEleven->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueTwelve->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_indicator->rotateAboutLocalAxisDeg(1,0,0,180);

    p_CommonData->p_expFrictionBox = new chai3d::cMesh();
}

void haptics_thread::InitDynamicBodies()
{
    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------
    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(p_CommonData->p_world);

    // Create an ODE Block
    p_CommonData->ODEBody1 = new cODEGenericBody(ODEWorld);
    p_CommonData->ODEBody2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    p_CommonData->p_dynamicBox1 = new chai3d::cMesh();
    p_CommonData->p_dynamicBox2 = new chai3d::cMesh();

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);

    //create ground
    ground = new chai3d::cMesh();
}

void haptics_thread::RenderDynamicBodies()
{
    delete ODEWorld;
    delete p_CommonData->ODEBody1;
    delete p_CommonData->ODEBody2;
    delete p_CommonData->p_dynamicBox1;
    delete p_CommonData->p_dynamicBox2;
    delete ODEGPlane0;
    delete ground;

    InitDynamicBodies();
    ODEWorld->deleteAllChildren();
    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    p_CommonData->p_world->addChild(ODEWorld);

    // give world gravity
    ODEWorld->setGravity(chai3d::cVector3d(0.0, 0.0, 9.81));
    // define damping properties
    ODEWorld->setAngularDamping(.005);
    ODEWorld->setLinearDamping(.002);

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0.05), chai3d::cVector3d(0.0, 0.0 ,-1.0));

    boxSize = 0.05;
    cCreateBox(p_CommonData->p_dynamicBox1, boxSize, boxSize, boxSize); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox2, boxSize, boxSize, boxSize); // make mesh a box
    p_CommonData->p_dynamicBox1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox2->createAABBCollisionDetector(toolRadius);

    //create a plane
    groundSize = 5.0;
    chai3d::cCreatePlane(ground, groundSize, groundSize);

    //position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0,0,0.05);

    // define some material properties for ground
    chai3d::cMaterial matGround;
    matGround.setStiffness(300);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setGrayLight();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    double staticFriction = 2.0;
    double dynamicFriction = staticFriction*0.9;

    // define material properties for object 1
    chai3d::cMaterial mat1;
    mat1.setBlueRoyal();
    mat1.setStiffness(600);
    mat1.setLateralStiffness(1000);
    mat1.setDynamicFriction(dynamicFriction);
    mat1.setStaticFriction(staticFriction);
    mat1.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox1->setMaterial(mat1);
    p_CommonData->p_dynamicBox1->setUseMaterial(true);

    // define material properties for object 2
    chai3d::cMaterial mat2;
    mat2.setRedCrimson();
    mat2.setStiffness(600);
    mat2.setLateralStiffness(1000);
    mat2.setDynamicFriction(dynamicFriction);
    mat2.setStaticFriction(staticFriction);
    mat2.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox2->setMaterial(mat2);
    p_CommonData->p_dynamicBox2->setUseMaterial(true);

    // add mesh to ODE object
    p_CommonData->ODEBody1->setImageModel(p_CommonData->p_dynamicBox1);
    p_CommonData->ODEBody2->setImageModel(p_CommonData->p_dynamicBox2);

    // create a dynamic model of the ODE object
    p_CommonData->ODEBody1->createDynamicBox(boxSize, boxSize, boxSize);
    p_CommonData->ODEBody2->createDynamicBox(boxSize, boxSize, boxSize);

    // set mass of box
    p_CommonData->ODEBody1->setMass(0.2);
    p_CommonData->ODEBody2->setMass(0.4);

    // set position of box
    p_CommonData->ODEBody1->setLocalPos(0,.1,0);
    p_CommonData->ODEBody2->setLocalPos(0,-.1,0);

    // setup tools for dynamic interaction
    m_tool0->enableDynamicObjects(true);
    m_tool1->enableDynamicObjects(true);

    world->addChild(ODEWorld);
    world->addChild(ground);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
    world->addChild(thumb);
}


void haptics_thread::RenderExpFriction()
{
    cCreateBox(p_CommonData->p_expFrictionBox, .09, .13, .01);
    p_CommonData->p_expFrictionBox->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_expFrictionBox->setLocalPos(0,0,0);
    p_CommonData->p_expFrictionBox->m_material->setStiffness(300);
    p_CommonData->p_expFrictionBox->m_material->setLateralStiffness(1580);
    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(0.4);
    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(0.4);
    world->addChild(p_CommonData->p_expFrictionBox);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}


void haptics_thread::RenderTwoFriction()
{    
    cCreateBox(p_CommonData->p_frictionBox1, .08, .08, .01); // make mesh a box
    cCreateBox(p_CommonData->p_frictionBox2, .08, .08, .01); // make mesh a box
    p_CommonData->p_frictionBox1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_frictionBox2->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_frictionBox1->setLocalPos(0,.05, 0);
    p_CommonData->p_frictionBox2->setLocalPos(0,-.05, 0);

    p_CommonData->p_frictionBox1->m_material->setStiffness(300);
    p_CommonData->p_frictionBox1->m_material->setLateralStiffness(600);
    p_CommonData->p_frictionBox1->m_material->setStaticFriction(0.25);
    p_CommonData->p_frictionBox1->m_material->setDynamicFriction(0.25*0.9);

    p_CommonData->p_frictionBox2->m_material->setStiffness(300);
    p_CommonData->p_frictionBox2->m_material->setLateralStiffness(600);
    p_CommonData->p_frictionBox2->m_material->setStaticFriction(0.8);
    p_CommonData->p_frictionBox2->m_material->setDynamicFriction(0.8*.9);

    world->addChild(p_CommonData->p_frictionBox1);
    world->addChild(p_CommonData->p_frictionBox2);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}

void haptics_thread::CommandSinPos(Eigen::Vector3d inputMotionAxis)
{
    // ----------------- START SINUSOID -----------------------------------
    // set the current time to "0"
    // scale to avoid abrupt starting input
    // make a dynamic ramp time of 2 periods
    double stillTime = 0.5;
    double rampTime = 6.0*1.0/p_CommonData->bandSinFreq;
    double currTime = p_CommonData->overallClock.getCurrentTimeSeconds() - p_CommonData->sinStartTime;
    double finalFreq = 40;

    // start recording after centering oscillation dies out
    if (currTime > 0.75*stillTime)
        p_CommonData->recordFlag = true;

    // case that we want to stay still at beginning
    if (currTime < stillTime)
    {
        p_CommonData->wearableDelta0->SetDesiredPos(p_CommonData->wearableDelta0->neutralPos);
    }    

    // case that we want to be ramping and then oscillating
    else if (currTime < ((20.0*1.0/p_CommonData->bandSinFreq) + stillTime))
    {
        p_CommonData->recordFlag = true;
        double scaledBandSinAmp = (currTime-stillTime)/rampTime*p_CommonData->bandSinAmp;
        if ((currTime - stillTime) > rampTime)
        {
            scaledBandSinAmp = p_CommonData->bandSinAmp;
        }

        Eigen::Vector3d sinPos = (scaledBandSinAmp*sin(2*PI*p_CommonData->bandSinFreq*(currTime-stillTime)))*inputMotionAxis + p_CommonData->wearableDelta0->neutralPos;
        p_CommonData->wearableDelta0->SetDesiredPos(sinPos);
    }

    // If time is greater than 20 periods + stillTime reset to neutral pos
    else if (currTime > ((20.0*1.0/p_CommonData->bandSinFreq) + stillTime))
    {
        p_CommonData->wearableDelta0->SetDesiredPos(p_CommonData->wearableDelta0->neutralPos);
        p_CommonData->recordFlag = false;
    }

    // If time is greater than 20 periods + 2*stillTime, pause and write to file
    if (currTime > ((20.0*1.0/p_CommonData->bandSinFreq) + 2*stillTime))
    {
        p_CommonData->wearableDelta0->TurnOffControl();
        WriteDataToFile();

        p_CommonData->bandSinFreq = p_CommonData->bandSinFreq + 0.2;
        p_CommonData->bandSinFreqDisp = p_CommonData->bandSinFreq;

        if (p_CommonData->bandSinFreq > finalFreq)
        {
            p_CommonData->currentControlState = idleControl;
        }

        Sleep(1000);
        p_CommonData->sinStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
    }
}

void haptics_thread::CommandCircPos(Eigen::Vector3d inputMotionAxis)
{
    double currTime = p_CommonData->overallClock.getCurrentTimeSeconds() - p_CommonData->circStartTime;
    double smallR = 1;
    double medR = 2;
    double largeR = 3;

    double moveStartTime = 1;
    double tactorSpeed = 5; // [mm/s]
    double circTime = 2*PI*p_CommonData->circRadius/tactorSpeed;

    double theta;
    double X;
    double Y;

    Eigen::Vector3d desPos;

    p_CommonData->recordFlag = false;
    // move to start small circle
    if (currTime < moveStartTime)
    {
        X = currTime/moveStartTime*p_CommonData->circRadius;
        Y = 0;
    }

    // perform small circle
    else if (currTime < (moveStartTime + 2*circTime))
    {
        p_CommonData->recordFlag = true;
        theta = 2*PI*(currTime - moveStartTime)/circTime;
        X = p_CommonData->circRadius*cos(theta);
        Y = p_CommonData->circRadius*sin(theta);
    }


    if (inputMotionAxis.x() == 1)
    {
        desPos[0] = 0;
        desPos[1] = X;
        desPos[2] = p_CommonData->wearableDelta0->neutralPos.z()+Y;
    }
    else if (inputMotionAxis.y() == 1)
    {
        desPos[0] = Y;
        desPos[1] = 0;
        desPos[2] = p_CommonData->wearableDelta0->neutralPos.z()+X;
    }
    else if (inputMotionAxis.z() == 1)
    {
        desPos[0] = X;
        desPos[1] = Y;
        desPos[2] = p_CommonData->wearableDelta0->neutralPos.z();
    }

    p_CommonData->wearableDelta0->SetDesiredPos(desPos);


    if (currTime > (moveStartTime + 2*circTime))
    {
        p_CommonData->wearableDelta0->TurnOffControl();
        WriteDataToFile();
        p_CommonData->currentControlState = idleControl;

        Sleep(1000);
    }
}

void haptics_thread::InitAccel()
{
#ifdef SENSORAY626
    // PERFORM INITIALIZATION OF SENSORAY FOR READING IN ADC
    // Allocate data structures. We allocate enough space for maximum possible
    // number of items (16) even though this example only has 3 items. Although
    // this is not required, it is the recommended practice to prevent programming
    // errors if your application ever modifies the number of items in the poll list.

    // Populate the poll list.
    poll_list[0] = 2 | RANGE_5V | EOPL; // Chan 2, ±5V range, mark as list end.
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
    chai3d::cVector3d returnVec;
    returnVec.set(0,0,databuf[0]);
    //qDebug() << returnVec.z();
    return returnVec;
#endif SENSORAY626

    chai3d::cVector3d emptyVec;
    return emptyVec; //if sensoray is not active, just return 0
}

void haptics_thread::WriteDataToFile()
{
    p_CommonData->recordFlag = false;
    QString tempFreq;
    tempFreq.setNum(p_CommonData->bandSinFreq);
    //write data to file when we are done
    std::ofstream file;
    file.open(p_CommonData->dir.toStdString() + "/" + p_CommonData->fileName.toStdString() + tempFreq.toStdString() + ".txt");
    for (int i=0; i < p_CommonData->debugData.size(); i++)
    {
        //[0] is distal finger, [1] is toward middle finger, [2] is away from finger pad
        file << p_CommonData->debugData[i].time << "," << " "
        << p_CommonData->debugData[i].pos[0] << "," << " "
        << p_CommonData->debugData[i].pos[1] << "," << " "
        << p_CommonData->debugData[i].pos[2] << "," << " "
        << p_CommonData->debugData[i].desiredPos[0] << "," << " "
        << p_CommonData->debugData[i].desiredPos[1] << "," << " "
        << p_CommonData->debugData[i].desiredPos[2] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForce[0] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForce[1] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForce[2] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[0] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[1] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[2] << "," << " "
        << p_CommonData->debugData[i].motorAngles[0] << "," << " "
        << p_CommonData->debugData[i].motorAngles[1] << "," << " "
        << p_CommonData->debugData[i].motorAngles[2] << "," << " "
        << p_CommonData->debugData[i].jointAngles[0] << "," << " "
        << p_CommonData->debugData[i].jointAngles[1] << "," << " "
        << p_CommonData->debugData[i].jointAngles[2] << "," << " "
        << p_CommonData->debugData[i].motorTorque[0] << "," << " "
        << p_CommonData->debugData[i].motorTorque[1] << "," << " "
        << p_CommonData->debugData[i].motorTorque[2] << "," << " "
        << p_CommonData->debugData[i].voltageOut[0] << "," << " "
        << p_CommonData->debugData[i].voltageOut[1] << "," << " "
        << p_CommonData->debugData[i].voltageOut[2] << "," << " "
        << p_CommonData->debugData[i].magTrackerPos0.x() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos0.y() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos0.z() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.x() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.y() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.z() << "," << " "
        << p_CommonData->debugData[i].accelSignal.x() << "," << " "
        << p_CommonData->debugData[i].accelSignal.y() << "," << " "
        << p_CommonData->debugData[i].accelSignal.z() << "," << " "
        << p_CommonData->debugData[i].tactileFeedback << "," << " "
        << p_CommonData->debugData[i].referenceFirst << "," << " "
        << p_CommonData->debugData[i].pairNo << "," << " "
        << p_CommonData->debugData[i].referenceFriction << "," << " "
        << p_CommonData->debugData[i].comparisonFriction << "," << " "
        << p_CommonData->debugData[i].subjectAnswer << "," << " "
        << p_CommonData->debugData[i].lumpLocation.x() << "," << " "
        << p_CommonData->debugData[i].lumpLocation.y() << "," << " "
        << p_CommonData->debugData[i].lineAngle << "," << " "
        << p_CommonData->debugData[i].lineAngleTruth << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,2) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,2) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,2) << "," << " "
        << std::endl;
    }
    file.close();
    p_CommonData->debugData.clear();
}

void haptics_thread::rotateTissueLineDisp(double angle)
{
    p_CommonData->p_indicator->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->indicatorRot = p_CommonData->indicatorRot + angle;
}

void haptics_thread::rotateTissueLine(double angle)
{
    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueSeven->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueEight->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueNine->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTen->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueEleven->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTwelve->rotateAboutLocalAxisDeg(0,0,-1,angle);
}

void haptics_thread::SetInitJointAngles()
{
    double moveTime = 2.0;
    if (p_CommonData->calibClock.getCurrentTimeSeconds() < moveTime)
    {
        double curr1, curr2, curr3, curr4, curr5, curr6;

        // handle device 0
        if (p_CommonData->device0Initing == true)
        {
            curr1 = 45*PI/180 - 57*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
            curr2 = 45*PI/180 - 57*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
            curr3 = 45*PI/180 - 57*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
        } else
        {
            curr1 = 45*PI/180;
            curr2 = 45*PI/180;
            curr3 = 45*PI/180;
        }
        p_CommonData->desJointInits0 << curr1, curr2, curr3;

        // handle device 1
        if (p_CommonData->device1Initing == true)
        {
            curr4 = 45*PI/180 - 45*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
            curr5 = 45*PI/180 - 45*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
            curr6 = 45*PI/180 - 45*p_CommonData->calibClock.getCurrentTimeSeconds()/moveTime*1*PI/180;
        } else
        {
            curr4 = 45*PI/180;
            curr5 = 45*PI/180;
            curr6 = 45*PI/180;
        }
        p_CommonData->desJointInits1 << curr4, curr5, curr6;
    } else
    {
        p_CommonData->device0Initing = false;
        p_CommonData->device1Initing = false;
    }


}

void haptics_thread::RenderPalpation()
{
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);

    double tissueNomStiffness = 300;
    double staticFriction = 0.6;
    double dynamicFriction = staticFriction*0.9;
    double vertOffset = 0.03;
    //----------------------------------------------Create Tissue One---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueOne);

    p_CommonData->p_tissueOne->setLocalPos(0,0,-.005+vertOffset);

    //load the object from file
    //cLoadFileOBJ(p_CommonData->p_tissueOne, "./Resources/tissue_1/tissue_1.obj");
    p_CommonData->p_tissueOne->loadFromFile("./Resources/tissue_1 OBJ/tissue_1.obj");

    // compute a boundary box
    p_CommonData->p_tissueOne->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueOne->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueOne->setTransparencyLevel(0.4, true, true);
    p_CommonData->p_tissueOne->setStiffness(tissueNomStiffness, true);
    p_CommonData->p_tissueOne->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Two---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueTwo);

    p_CommonData->p_tissueTwo->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueTwo->loadFromFile("./Resources/tissue_2 OBJ/tissue_2.obj");

    // compute a boundary box
    p_CommonData->p_tissueTwo->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueTwo->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueTwo->setStiffness(tissueNomStiffness*1.05, true);
    p_CommonData->p_tissueTwo->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Three---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueThree);

    p_CommonData->p_tissueThree->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueThree->loadFromFile("./Resources/tissue_3 OBJ/tissue_3.obj");

    // compute a boundary box
    p_CommonData->p_tissueThree->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueThree->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueThree->setStiffness(tissueNomStiffness*1.25, true);
    p_CommonData->p_tissueThree->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Four---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueFour);

    p_CommonData->p_tissueFour->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueFour->loadFromFile("./Resources/tissue_4 OBJ/tissue_4.obj");

    // compute a boundary box
    p_CommonData->p_tissueFour->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueFour->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueFour->setStiffness(tissueNomStiffness*1.1, true);
    p_CommonData->p_tissueFour->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Five---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueFive);

    p_CommonData->p_tissueFive->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueFive->loadFromFile("./Resources/tissue_5 OBJ/tissue_5.obj");

    // compute a boundary box
    p_CommonData->p_tissueFive->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueFive->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueFive->setStiffness(tissueNomStiffness*1.15, true);
    p_CommonData->p_tissueFive->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Six---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueSix);

    p_CommonData->p_tissueSix->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueSix->loadFromFile("./Resources/tissue_6 OBJ/tissue_6.obj");

    // compute a boundary box
    p_CommonData->p_tissueSix->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueSix->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueSix->setStiffness(tissueNomStiffness*1.2, true);
    p_CommonData->p_tissueSix->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Seven---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueSeven);

    p_CommonData->p_tissueSeven->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueSeven->loadFromFile("./Resources/tissue_7 OBJ/tissue_7.obj");

    // compute a boundary box
    p_CommonData->p_tissueSeven->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueSeven->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueSeven->setStiffness(tissueNomStiffness*1.25, true);
    p_CommonData->p_tissueSeven->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Eight---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueEight);

    p_CommonData->p_tissueEight->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueEight->loadFromFile("./Resources/tissue_8 OBJ/tissue_8.obj");

    // compute a boundary box
    p_CommonData->p_tissueEight->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueEight->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueEight->setStiffness(tissueNomStiffness*1.3, true);
    p_CommonData->p_tissueEight->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Nine---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueNine);

    p_CommonData->p_tissueNine->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueNine->loadFromFile("./Resources/tissue_9 OBJ/tissue_9.obj");

    // compute a boundary box
    p_CommonData->p_tissueNine->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueNine->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueNine->setStiffness(tissueNomStiffness*1.35, true);
    p_CommonData->p_tissueNine->setFriction(staticFriction, dynamicFriction, TRUE);
    //----------------------------------------------Create Tissue Ten---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueTen);

    p_CommonData->p_tissueTen->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueTen->loadFromFile("./Resources/tissue_10 OBJ/tissue_10.obj");

    // compute a boundary box
    p_CommonData->p_tissueTen->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueTen->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueTen->setStiffness(tissueNomStiffness*1.4, true);
    p_CommonData->p_tissueTen->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Eleven---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueEleven);

    p_CommonData->p_tissueEleven->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueEleven->loadFromFile("./Resources/tissue_11 OBJ/tissue_11.obj");

    // compute a boundary box
    p_CommonData->p_tissueEleven->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueEleven->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueEleven->setStiffness(tissueNomStiffness*1.45, true);
    p_CommonData->p_tissueEleven->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Twelve---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueTwelve);

    p_CommonData->p_tissueTwelve->setLocalPos(0,0,-.025+vertOffset);

    //load the object from file
    p_CommonData->p_tissueTwelve->loadFromFile("./Resources/tissue_12 OBJ/tissue_12.obj");

    // compute a boundary box
    p_CommonData->p_tissueTwelve->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_tissueTwelve->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_tissueTwelve->setStiffness(tissueNomStiffness*1.5, true);
    p_CommonData->p_tissueTwelve->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Indicator---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_indicator);

    p_CommonData->p_indicator->setLocalPos(0,0,-.026+vertOffset);

    //load the object from file
    p_CommonData->p_indicator->loadFromFile("./Resources/tissue_indicator OBJ/tissue_indicator.obj");

    p_CommonData->p_indicator->setHapticEnabled(false);
    chai3d::cMaterial mat0;
    mat0.setRed();
    p_CommonData->p_indicator->setMaterial(mat0);
    p_CommonData->p_indicator->setTransparencyLevel(1);

}




