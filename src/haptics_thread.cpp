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
    InitEnvironments();
    //InitDynamicBodies();

    // GENERAL HAPTICS INITS=================================
    // Ensure the device is not controlling to start
    p_CommonData->wearableDelta->TurnOffControl();
    p_CommonData->neutralPos << 0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180);
    p_CommonData->wearableDelta->SetDesiredPos(p_CommonData->neutralPos); // kinematic neutral position

    p_CommonData->Kp = 0; //these are set by the window sliders
    p_CommonData->Kd = 0; //these are set by the window sliders
    p_CommonData->jointKp = 1700.0;
    p_CommonData->jointKd = 10;

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
    rateClock.setTimeoutPeriodSeconds(0.00000000013);
    rateClock.start(true);

    // setup the clock that will enable display of the haptic rate
    rateDisplayClock.reset();
    rateDisplayClock.setTimeoutPeriodSeconds(1.0);
    rateDisplayClock.start(true);

    // setup the overall program time clock
    p_CommonData->overallClock.reset();
    p_CommonData->overallClock.start(true);

    currTime = 0;
    lastTime = 0;

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
            Eigen::Vector3d inputAxis(0,1,0); // input axis for sin control and circ control modes
            switch(p_CommonData->currentControlState)
            {

            case idleControl:
                UpdateVRGraphics();
                p_CommonData->wearableDelta->TurnOffControl();
                break;

            case VRControlMode:
                UpdateVRGraphics();
                ComputeVRDesiredDevicePos();
                p_CommonData->wearableDelta->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case sliderControlMode:
                UpdateVRGraphics();
                p_CommonData->wearableDelta->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case sinControlMode:
                UpdateVRGraphics();                
                CommandSinPos(inputAxis);
                p_CommonData->wearableDelta->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
                break;

            case circControlMode:
                UpdateVRGraphics();
                CommandCircPos(inputAxis);
                p_CommonData->wearableDelta->JointController(p_CommonData->jointKp, p_CommonData->jointKd);
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
            if(recordDataCounter == 10)
            {
                recordDataCounter = 0;
                if(p_CommonData->recordFlag == true)
                {
                    RecordData();
                    //qDebug() << "indicator Rot: " << p_CommonData->indicatorRot << "tissue rot: " << p_CommonData->tissueRot;
                }
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
    // Update camera Pos
    double xPos = p_CommonData->camRadius*cos(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double yPos = p_CommonData->camRadius*sin(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double zPos = p_CommonData->camRadius*cos(p_CommonData->polar*PI/180.0);
    p_CommonData->cameraPos.set(xPos, yPos, zPos);

    //update camera parameters
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

        case experimentPalpation:
            world->clearAllChildren();
            RenderExpPalpation();
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

        case hump:
            world->clearAllChildren();
            RenderHump();
            break;

        case hoopHump:
            world->clearAllChildren();
            RenderHoopHump();
            break;

        case dynamicBodies:
            world->clearAllChildren();
            RenderDynamicBodies();
            break;

        case paperEnvironment:
            world->clearAllChildren();
            RenderPaper();
            break;
        }
    }


    // mainwindow makes the line visible after trial, this makes it opaque again and starts the next trial
    if(p_CommonData->palpPostTrialClock.timeoutOccurred())
    {
        p_CommonData->palpPostTrialClock.stop();
        p_CommonData->palpPostTrialClock.reset();

        // increment trial no
        p_CommonData->trialNo = p_CommonData->trialNo + 1;

        QString type = (p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "type", NULL /*default*/));

        if(type == "break")
        {
            p_CommonData->currentExperimentState = palpationLineBreak;
            p_CommonData->p_tissueOne->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(0, true);
        }

        else if(type == "trial")
        {
            p_CommonData->currentExperimentState = palpationLineTrial;
            // rotate line to new location
            double lastRotation = p_CommonData->tissueRot;
            p_CommonData->tissueRot = atoi(p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Angles", NULL /*default*/));

            rotateTissueLine(-lastRotation);
            rotateTissueLine(p_CommonData->tissueRot);

            p_CommonData->p_tissueOne->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(0, true);
            p_CommonData->recordFlag = true;
        }

        else if (type == "training")
        {
            p_CommonData->currentExperimentState = palpationLineTrial;
            // rotate line to new location
            double lastRotation = p_CommonData->tissueRot;
            p_CommonData->tissueRot = atoi(p_CommonData->palpationLineProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo)).toStdString().c_str(), "Angles", NULL /*default*/));

            rotateTissueLine(-lastRotation);
            rotateTissueLine(p_CommonData->tissueRot);
            p_CommonData->p_tissueSix->setTransparencyLevel(0, true);

            p_CommonData->recordFlag = true;
        }
        else if (type == "end")
        {
            p_CommonData->currentExperimentState = end;
            p_CommonData->p_tissueOne->setTransparencyLevel(1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(0, true);
        }

        // rotate tissue line indicator back to 0
        double lastDispRotation = p_CommonData->indicatorRot;
        rotateTissueLineDisp(-lastDispRotation);
    }

//    // mainwindow makes the lump visible after trial, this makes it opaque again and starts the next trial
//    if(p_CommonData->palpPostTrialClock.timeoutOccurred())
//    {
//        p_CommonData->palpPostTrialClock.stop();
//        p_CommonData->palpPostTrialClock.reset();
//        // increment trial no
//        p_CommonData->trialNo = p_CommonData->trialNo + 1;
//         // make tissue opaque again if done with visible training trials
//        if (p_CommonData->trialNo > 3)
//        {
//            p_CommonData->p_tissueCyl->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueBox->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueLump->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueLumpCenter->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueLumpCenter1->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueLumpCenter2->setTransparencyLevel(1.0, true);
//            p_CommonData->p_tissueLumpCenter3->setTransparencyLevel(1.0, true);
//        }
//        // move lump to a different location
//        double tissueRad = 0.08; double lumpRad = tissueRad*0.15;
//        double boxWidth = 2.0*tissueRad;
//        double boxDepth = 0.1;
//        double boxHeight = 0.05;
//        double widthMax = boxWidth/2-lumpRad; double widthMin = -boxWidth/2+lumpRad;
//        double depthMax = boxDepth/2-lumpRad; double depthMin = -boxDepth/2+lumpRad;
//        double y = ((double) rand()*(widthMax-widthMin)/(double)RAND_MAX+widthMin);
//        double x = ((double) rand()*(depthMax-depthMin)/(double)RAND_MAX+depthMin);
//        p_CommonData->p_tissueLump->setLocalPos(x,y,-0.0000001);
//        p_CommonData->p_tissueLumpCenter->setLocalPos(x,y,-0.00000011);
//        p_CommonData->p_tissueLumpCenter1->setLocalPos(x,y,-0.00000012);
//        p_CommonData->p_tissueLumpCenter2->setLocalPos(x,y,-0.00000013);
//        p_CommonData->p_tissueLumpCenter3->setLocalPos(x,y,-0.00000014);
//        p_CommonData->recordFlag = true;
//        p_CommonData->currentExperimentState = palpationTrial;
//    }

    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    // update position and orientation of tool (and sphere that represents tool)
    m_tool0->updatePose();

    // get position and rotation of the magTracker
    p_CommonData->chaiMagDevice0->getPosition(position0);
    p_CommonData->chaiMagDevice0->getRotation(rotation0);

    // set the visual representation to match
    m_curSphere0->setLocalPos(position0);
    m_curSphere0->setLocalRot(rotation0);
    m_tool0->computeInteractionForces();

    // update position of finger to stay on proxy point
    // finger axis are not at fingerpad, so we want a translation along fingertip z axis
    chai3d::cVector3d fingerOffset(0,-0.006,0);
    fingerRotation0 = rotation0;
    fingerRotation0.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation0.rotateAboutLocalAxisDeg(1,0,0,90);
    finger->setLocalRot(fingerRotation0);
    finger->setLocalPos(m_tool0->m_hapticPoint->getGlobalPosProxy() + fingerRotation0*fingerOffset);

    //use this if two tools (haptic proxies) are desired
//    m_tool1->updatePose();
//    p_CommonData->chaiMagDevice1->getPosition(position1);
//    p_CommonData->chaiMagDevice1->getRotation(rotation1);
//    m_curSphere1->setLocalPos(position1);
//    m_curSphere1->setLocalRot(rotation1);
//    m_tool1->computeInteractionForces();

    // perform our dynamic body updates if we are in a dynamic environment
    if((p_CommonData->currentEnvironmentState == dynamicBodies) | (p_CommonData->currentEnvironmentState == paperEnvironment))
    {
        currTime = p_CommonData->overallClock.getCurrentTimeSeconds();
        double timeInterval = currTime - lastTime;
        if(timeInterval > 0.01) timeInterval = 0.01;
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
        ODEWorld->updateDynamics(timeInterval);
        lastTime = currTime;
    }
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

    deviceLastForceRecord << deviceLastComputedForce0.x(),deviceLastComputedForce0.y(),deviceLastComputedForce0.z();
    globalLastForceRecord << lastComputedForce0.x(), lastComputedForce0.y(), lastComputedForce0.z();

    //convert device "force" to a mapped position
    double forceToPosMult = 1.0/1.588; // based on lateral stiffness of finger (averaged directions from Gleeson paper) (1.588 N/mm)
    chai3d::cVector3d desiredPosMovement = forceToPosMult*deviceLastComputedForce0; //this is only for lateral

    // don't allow the tactor to move away from finger

    double vertPosMovement = desiredPosMovement.z();
    if(vertPosMovement > 0)
        vertPosMovement = 0;
    /*
    if(deviceLastComputedForce0.z() > 0)
        vertPosMovement = log(8.6736*deviceLastComputedForce0.z()+1.0); //linear fit exp data from normal displacement paper
    else
        vertPosMovement = -log(8.6736*abs(deviceLastComputedForce0.z())+1.0);*/ // was too abrupt at first movement

    /*double vertForceToPosMult = 1.0/0.8676; // based on vertical stiffness of finger (3.47N/mm from WHC paper)
    double vertPosMovement = vertForceToPosMult*deviceLastComputedForce0.z();*/ //was still too aggresive with vertical

    Eigen::Vector3d neutralPos = p_CommonData->neutralPos;
    Eigen::Vector3d desiredPos(3);
    desiredPos << desiredPosMovement.x()+neutralPos[0], desiredPosMovement.y()+neutralPos[1], vertPosMovement+neutralPos[2];

    // if the experimental condition is no feedback, tell it to move to neutral pos
    if(p_CommonData->tactileFeedback == 0)
    {
        desiredPos << neutralPos[0], neutralPos[1], neutralPos[2];
    }

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
    dataRecorder.VRInteractionForce = deviceLastForceRecord; // last force on tool0
    dataRecorder.VRInteractionForceGlobal = globalLastForceRecord; // last force on tool0 in global coords
    dataRecorder.motorTorque = p_CommonData->wearableDelta->motorTorques;
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
    p_CommonData->cameraPos.set(0.18, 0.0, 0);
    p_CommonData->lookatPos.set(0.0, 0.0, 0.0);
    p_CommonData->upVector.set(0.0, 0.0, -1.0);
    p_CommonData->p_camera->set( p_CommonData->cameraPos,//(0.25, 0, -.25),    // camera position (eye)
                                 p_CommonData->lookatPos,    // lookat position (target)
                                 p_CommonData->upVector);   // direction of the "up" vector

    p_CommonData->azimuth = 0.0;
    p_CommonData->polar = 140.0;
    p_CommonData->camRadius = 0.15;

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
    //m_tool0->enableDynamicObjects(true);
    m_tool0->start();

    //uncomment this if we want to use 2 tools
//    m_tool1 = new chai3d::cToolCursor(world); // create a 3D tool
//    world->addChild(m_tool1); //insert the tool into the world
//    m_tool1->setRadius(toolRadius);
//    m_tool1->setHapticDevice(p_CommonData->chaiMagDevice1); // connect the haptic device to the tool
//    //m_tool1->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
//    //m_tool1->enableDynamicObjects(true);
//    m_tool1->start();

    // Can use this to show frames on tool if so desired
    //create a sphere to represent the tool
    m_curSphere0 = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere0);
    m_curSphere0->m_material->setGrayDarkSlate();
    m_curSphere0->setShowFrame(true);
    m_curSphere0->setFrameSize(0.05);

//    m_curSphere1 = new chai3d::cShapeSphere(toolRadius);
//    world->addChild(m_curSphere1);
//    m_curSphere1->m_material->setBlueAqua();
//    m_curSphere1->setShowFrame(true);
//    m_curSphere1->setFrameSize(0.05);


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------
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
    //finger->computeBoundaryBox(true); //compute a boundary box
    finger->setUseVertexColors(true);
    chai3d::cColorf fingerColor;
    fingerColor.setBrownSandy();
    finger->setVertexColor(fingerColor);
    finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
    finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    finger->m_material->m_specular.set(1.0, 1.0, 1.0);
    finger->setUseMaterial(true);
    finger->setHapticEnabled(false);
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
    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(1,0,0,180);
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(1,0,0,180);

    p_CommonData->p_hump = new chai3d::cMultiMesh();
    p_CommonData->p_hoopHump = new chai3d::cMultiMesh();
    p_CommonData->p_hump->rotateAboutGlobalAxisDeg(1,0,0,-90);
    p_CommonData->p_hoopHump->rotateAboutGlobalAxisDeg(1,0,0,-90);

    p_CommonData->p_expFrictionBox = new chai3d::cMesh();

    p_CommonData->p_tissueCyl = new chai3d::cMesh();
    p_CommonData->p_tissueLump = new chai3d::cMesh();
    p_CommonData->p_tissueLumpCenter = new chai3d::cMesh();
    p_CommonData->p_tissueLumpCenter1 = new chai3d::cMesh();
    p_CommonData->p_tissueLumpCenter2 = new chai3d::cMesh();
    p_CommonData->p_tissueLumpCenter3 = new chai3d::cMesh();
    p_CommonData->p_tissueBox = new chai3d::cMesh();
}

void haptics_thread::InitDynamicBodies()
{
    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);
    //give world gravity
    ODEWorld->setGravity(chai3d::cVector3d(0.0, 0.0, 9.81));
    // define damping properties
    ODEWorld->setAngularDamping(.2);
    ODEWorld->setLinearDamping(.02);

    // Create an ODE Block
    p_CommonData->ODEBody0 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    p_CommonData->p_dynamicBox = new chai3d::cMesh();

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0.05), chai3d::cVector3d(0.0, 0.0,-1.0));

    //create ground
    ground = new chai3d::cMesh();

    //create a plane
    double groundSize = 5.0;
    chai3d::cCreatePlane(ground, groundSize, groundSize);

    //position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0,0,0.05);

    //define some material properties
    chai3d::cMaterial matGround;
    matGround.setStiffness(300);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setGrayLight();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);
}

void haptics_thread::RenderDynamicBodies()
{
    ODEWorld->deleteAllChildren();
    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);
    //give world gravity
    ODEWorld->setGravity(chai3d::cVector3d(0.0, 0.0, 9.81));
    // define damping properties
    ODEWorld->setAngularDamping(.02);
    ODEWorld->setLinearDamping(.00002);

    // Create an ODE Block
    p_CommonData->ODEBody0 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    p_CommonData->p_dynamicBox = new chai3d::cMesh();

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0.05), chai3d::cVector3d(0.0, 0.0 ,-1.0));

    //create ground
    ground = new chai3d::cMesh();

    //create a plane
    double groundSize = 5.0;
    chai3d::cCreatePlane(ground, groundSize, groundSize);

    //position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0,0,0.05);

    //define some material properties
    chai3d::cMaterial matGround;
    matGround.setStiffness(300);
    matGround.setDynamicFriction(0.2);
    matGround.setStaticFriction(0.0);
    matGround.setGrayLight();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);

    double boxSize = 0.05;
    cCreateBox(p_CommonData->p_dynamicBox, boxSize, boxSize, boxSize); // make mesh a box

    p_CommonData->p_dynamicBox->createAABBCollisionDetector(toolRadius);
    chai3d::cMaterial mat0;
    mat0.setBlueRoyal();
    mat0.setStiffness(300);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    p_CommonData->p_dynamicBox->setMaterial(mat0);

    // add mesh to ODE object
    p_CommonData->ODEBody0->setImageModel(p_CommonData->p_dynamicBox);

    // create a dynamic model of the ODE object
    p_CommonData->ODEBody0->createDynamicBox(boxSize, boxSize, boxSize);

    // set mass of box
    p_CommonData->ODEBody0->setMass(0.05);

    // set position of box
    p_CommonData->ODEBody0->setLocalPos(0,0,0);

    world->addChild(ODEWorld);
    world->addChild(ground);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}

void haptics_thread::RenderPaper()
{
    ODEWorld->deleteAllChildren();
    //--------------------------------------------------------------------------
    // CREATING ODE World and Objects
    //--------------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);
    //give world gravity
    ODEWorld->setGravity(chai3d::cVector3d(0.0, 0.0, 0));
    // define damping properties
    ODEWorld->setAngularDamping(.02);
    ODEWorld->setLinearDamping(.00000002);

    // Create an ODE Block
    p_CommonData->ODEBody0 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    p_CommonData->p_dynamicBox = new chai3d::cMesh();

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0.05), chai3d::cVector3d(0.0, 0.0 ,-1.0));

    //create ground
    ground = new chai3d::cMesh();

    //create a plane
    double groundSize = 5.0;
    chai3d::cCreatePlane(ground, groundSize, groundSize);

    //position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0,0,0.05);

    //define some material properties
    chai3d::cMaterial matGround;
    matGround.setStiffness(300);
    matGround.setDynamicFriction(1);
    matGround.setStaticFriction(1);
    matGround.setGrayLight();
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);

    double boxSize = 0.05;
    cCreateBox(p_CommonData->p_dynamicBox, 2.0*boxSize, 2.0*boxSize, .005); // make mesh a box

    p_CommonData->p_dynamicBox->createAABBCollisionDetector(toolRadius);
    chai3d::cMaterial mat0;
    mat0.setBlueRoyal();
    mat0.setStiffness(300);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    p_CommonData->p_dynamicBox->setMaterial(mat0);

    // add mesh to ODE object
    p_CommonData->ODEBody0->setImageModel(p_CommonData->p_dynamicBox);

    // create a dynamic model of the ODE object
    p_CommonData->ODEBody0->createDynamicBox(2.0*boxSize, 2.0*boxSize, .005);

    // set mass of box
    p_CommonData->ODEBody0->setMass(0.05);

    // set position of box
    p_CommonData->ODEBody0->setLocalPos(0,0,0);

    world->addChild(ODEWorld);
    world->addChild(ground);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);

}

void haptics_thread::RenderHump()
{
    p_CommonData->p_hump->loadFromFile("./Resources/Hump.obj");

    p_CommonData->p_hump->computeBoundaryBox(true); //compute a boundary box

    // compute collision detection algorithm
    p_CommonData->p_hump->createAABBCollisionDetector(toolRadius);

    chai3d::cColorf humpColor;
    humpColor.setBlueDeepSky();
    p_CommonData->p_hump->setVertexColor(humpColor);
    p_CommonData->p_hump->m_material->m_ambient.set(0.1, 0.1, 0.1);
    p_CommonData->p_hump->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    p_CommonData->p_hump->m_material->m_specular.set(1.0, 1.0, 1.0);
    p_CommonData->p_hump->setUseMaterial(true);
    p_CommonData->p_hump->setStiffness(200);
    p_CommonData->p_hump->setFriction(0.5, 0.5, true);
    world->addChild(p_CommonData->p_hump);

    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}

void haptics_thread::RenderHoopHump()
{
    p_CommonData->p_hoopHump->loadFromFile("./Resources/HumpHoopImported.obj");

    p_CommonData->p_hoopHump->computeBoundaryBox(true); //compute a boundary box

    // compute collision detection algorithm
    p_CommonData->p_hoopHump->createAABBCollisionDetector(toolRadius);

    chai3d::cColorf hoopHumpColor;
    hoopHumpColor.setBlueDeepSky();
    p_CommonData->p_hoopHump->setVertexColor(hoopHumpColor);
    p_CommonData->p_hoopHump->m_material->m_ambient.set(0.1, 0.1, 0.1);
    p_CommonData->p_hoopHump->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    p_CommonData->p_hoopHump->m_material->m_specular.set(1.0, 1.0, 1.0);
    p_CommonData->p_hoopHump->setUseMaterial(true);
    p_CommonData->p_hoopHump->setStiffness(200);
    p_CommonData->p_hoopHump->setFriction(0.5, 0.5, true);

    world->addChild(p_CommonData->p_hoopHump);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}

void haptics_thread::RenderExpFriction()
{
    cCreateBox(p_CommonData->p_expFrictionBox, .065, .13, .01);
    p_CommonData->p_expFrictionBox->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_expFrictionBox->setLocalPos(0,0,0);
    p_CommonData->p_expFrictionBox->m_material->setStiffness(200);
    p_CommonData->p_expFrictionBox->m_material->setStaticFriction(0.4);
    p_CommonData->p_expFrictionBox->m_material->setDynamicFriction(0.4);
    world->addChild(p_CommonData->p_expFrictionBox);
    world->addChild(m_tool0);
    world->addChild(m_tool1);
    world->addChild(finger);
}

void haptics_thread::RenderExpPalpation()
{
    // define sizes of the palpation tissue
    double tissueRad = 0.08;
    double lumpRad = tissueRad*0.15;
    double lumpCenterRad = tissueRad*.15*0.9;
    double lumpCenterRad1 = tissueRad*.15*0.8;
    double lumpCenterRad2 = tissueRad*.15*0.7;
    double lumpCenterRad3 = tissueRad*.15*0.6;

    double boxWidth = 2.0*tissueRad;
    double boxDepth = 0.1;
    double boxHeight = 0.05;

    double tissueNomStiffness = 200; double nomFriction = 0.5;

    // create the meshes for the tissue rendering
//    cCreateCylinder(p_CommonData->p_tissueCyl, 0.05, tissueRad);
//    p_CommonData->p_tissueCyl->createAABBCollisionDetector(toolRadius);
//    p_CommonData->p_tissueCyl->setLocalPos(0,0,0);
//    p_CommonData->p_tissueCyl->m_material->setStiffness(tissueNomStiffness);
//    p_CommonData->p_tissueCyl->m_material->setStaticFriction(nomFriction);
//    p_CommonData->p_tissueCyl->m_material->setDynamicFriction(nomFriction*0.9);
//    p_CommonData->p_tissueCyl->m_material->setBrownTan();

    // create the main body of tissue (box rather than cylinder)
    cCreateBox(p_CommonData->p_tissueBox, boxDepth, boxWidth, boxHeight);
    p_CommonData->p_tissueBox->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueBox->setLocalPos(0,0,boxHeight/2);
    p_CommonData->p_tissueBox->m_material->setStiffness(tissueNomStiffness);
    p_CommonData->p_tissueBox->m_material->setStaticFriction(nomFriction);
    p_CommonData->p_tissueBox->m_material->setDynamicFriction(nomFriction*0.9);
    p_CommonData->p_tissueBox->m_material->setBrownTan();

    cCreateCylinder(p_CommonData->p_tissueLump, 0.05, lumpRad);
    p_CommonData->p_tissueLump->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueLump->m_material->setStiffness(tissueNomStiffness*1.1);
    p_CommonData->p_tissueLump->m_material->setStaticFriction(nomFriction);
    p_CommonData->p_tissueLump->m_material->setDynamicFriction(nomFriction*0.9);
    p_CommonData->p_tissueLump->m_material->setBrownTan();

    cCreateCylinder(p_CommonData->p_tissueLumpCenter, 0.05, lumpCenterRad);
    p_CommonData->p_tissueLumpCenter->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueLumpCenter->m_material->setStiffness(tissueNomStiffness*1.2);
    p_CommonData->p_tissueLumpCenter->m_material->setStaticFriction(0.5);
    p_CommonData->p_tissueLumpCenter->m_material->setDynamicFriction(0.5*0.9);
    p_CommonData->p_tissueLumpCenter->m_material->setBrownTan();

    cCreateCylinder(p_CommonData->p_tissueLumpCenter1, 0.05, lumpCenterRad1);
    p_CommonData->p_tissueLumpCenter1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueLumpCenter1->m_material->setStiffness(tissueNomStiffness*1.3);
    p_CommonData->p_tissueLumpCenter1->m_material->setStaticFriction(0.5);
    p_CommonData->p_tissueLumpCenter1->m_material->setDynamicFriction(0.5*0.9);
    p_CommonData->p_tissueLumpCenter1->m_material->setBrownTan();

    cCreateCylinder(p_CommonData->p_tissueLumpCenter2, 0.05, lumpCenterRad2);
    p_CommonData->p_tissueLumpCenter2->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueLumpCenter2->m_material->setStiffness(tissueNomStiffness*1.4);
    p_CommonData->p_tissueLumpCenter2->m_material->setStaticFriction(0.5);
    p_CommonData->p_tissueLumpCenter2->m_material->setDynamicFriction(0.5*0.9);
    p_CommonData->p_tissueLumpCenter2->m_material->setBrownTan();

    cCreateCylinder(p_CommonData->p_tissueLumpCenter3, 0.05, lumpCenterRad3);
    p_CommonData->p_tissueLumpCenter3->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_tissueLumpCenter3->m_material->setStiffness(tissueNomStiffness*1.5);
    p_CommonData->p_tissueLumpCenter3->m_material->setStaticFriction(0.5);
    p_CommonData->p_tissueLumpCenter3->m_material->setDynamicFriction(0.5*0.9);
    p_CommonData->p_tissueLumpCenter3->m_material->setBrownTan();

    // put the higher stiffness "lump" in a random location (cyl)
//    srand (time(NULL));
//    double max = tissueRad - lumpRad; double min = 0;
//    double angMax = 2*PI; double angMin = 0;
//    double rad = ((double) rand()*(max-min)/(double)RAND_MAX+min);
//    double ang = ((double) rand()*(angMax-angMin)/(double)RAND_MAX+angMin);
//    p_CommonData->p_tissueLump->setLocalPos(rad*cos(ang),rad*sin(ang),-0.0000001);
//    p_CommonData->p_tissueLumpCenter->setLocalPos(rad*cos(ang),rad*sin(ang),-0.00000011);
//    p_CommonData->p_tissueLumpCenter1->setLocalPos(rad*cos(ang),rad*sin(ang),-0.00000012);
//    p_CommonData->p_tissueLumpCenter2->setLocalPos(rad*cos(ang),rad*sin(ang),-0.00000013);
//    p_CommonData->p_tissueLumpCenter3->setLocalPos(rad*cos(ang),rad*sin(ang),-0.00000014);

    // put the higher stiffness "lump" in a random location (box)
    srand (time(NULL));
    double widthMax = boxWidth/2-lumpRad; double widthMin = -boxWidth/2+lumpRad;
    double depthMax = boxDepth/2-lumpRad; double depthMin = -boxDepth/2+lumpRad;
    double y = ((double) rand()*(widthMax-widthMin)/(double)RAND_MAX+widthMin);
    double x = ((double) rand()*(depthMax-depthMin)/(double)RAND_MAX+depthMin);
    p_CommonData->p_tissueLump->setLocalPos(x,y,-0.0000001);
    p_CommonData->p_tissueLumpCenter->setLocalPos(x,y,-0.00000011);
    p_CommonData->p_tissueLumpCenter1->setLocalPos(x,y,-0.00000012);
    p_CommonData->p_tissueLumpCenter2->setLocalPos(x,y,-0.00000013);
    p_CommonData->p_tissueLumpCenter3->setLocalPos(x,y,-0.00000014);

    world->addChild(p_CommonData->p_tissueBox);
    world->addChild(p_CommonData->p_tissueLump);
    world->addChild(p_CommonData->p_tissueLumpCenter);
    world->addChild(p_CommonData->p_tissueLumpCenter1);
    world->addChild(p_CommonData->p_tissueLumpCenter2);
    world->addChild(p_CommonData->p_tissueLumpCenter3);
    world->addChild(m_tool0);
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

    p_CommonData->p_frictionBox1->m_material->setStiffness(200);
    p_CommonData->p_frictionBox1->m_material->setStaticFriction(0.4);
    p_CommonData->p_frictionBox1->m_material->setDynamicFriction(0.4*0.9);

    p_CommonData->p_frictionBox2->m_material->setStiffness(200);
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
        p_CommonData->wearableDelta->SetDesiredPos(p_CommonData->neutralPos);        
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

        Eigen::Vector3d sinPos = (scaledBandSinAmp*sin(2*PI*p_CommonData->bandSinFreq*(currTime-stillTime)))*inputMotionAxis + p_CommonData->neutralPos;
        p_CommonData->wearableDelta->SetDesiredPos(sinPos);
    }

    // If time is greater than 20 periods + stillTime reset to neutral pos
    else if (currTime > ((20.0*1.0/p_CommonData->bandSinFreq) + stillTime))
    {
        p_CommonData->wearableDelta->SetDesiredPos(p_CommonData->neutralPos);
        p_CommonData->recordFlag = false;
    }

    // If time is greater than 20 periods + 2*stillTime, pause and write to file
    if (currTime > ((20.0*1.0/p_CommonData->bandSinFreq) + 2*stillTime))
    {
        p_CommonData->wearableDelta->TurnOffControl();
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
    double circleR = 3.0;
    double currTime = p_CommonData->overallClock.getCurrentTimeSeconds() - p_CommonData->circStartTime;
    double revTime = 2.5;
    double waitTime = 0.1;
    double theta = 2.0*PI/revTime*currTime; //the angle grows as one rev every revTime seconds
    Eigen::Vector3d desPos(0,0,p_CommonData->neutralPos.z());

    double rampTime = 2.0*revTime; //ramp the amplitude over 2 revolutions
    double finalTime = 10.0*revTime; // ending time, then write to file
    double amp;
    p_CommonData->recordFlag = true;

    if (currTime < waitTime)
    {
        amp = 0;
    }
    else if (currTime < rampTime)
    {
        amp = circleR*currTime/rampTime;
    }
    else if (currTime > rampTime)
    {
        amp = circleR;        
    }

    double X = amp*cos(theta);
    double Y = amp*sin(theta);

    if (inputMotionAxis.x() == 1)
    {
        desPos[0] = 0;
        desPos[1] = X;
        desPos[2] = p_CommonData->neutralPos.z()+Y;
    }
    else if (inputMotionAxis.y() == 1)
    {
        desPos[0] = Y;
        desPos[1] = 0;
        desPos[2] = p_CommonData->neutralPos.z()+X;
    }
    else if (inputMotionAxis.z() == 1)
    {
        desPos[0] = X;
        desPos[1] = Y;
        desPos[2] = p_CommonData->neutralPos.z();
    }

    p_CommonData->wearableDelta->SetDesiredPos(desPos);


    if (currTime > finalTime)
    {
        p_CommonData->wearableDelta->TurnOffControl();
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
        << std::endl;
    }
    file.close();
    p_CommonData->debugData.clear();
}

void haptics_thread::rotateTissueLineDisp(double angle)
{
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->indicatorRot = p_CommonData->indicatorRot + angle;
}

void haptics_thread::rotateTissueLine(double angle)
{
    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(0,0,-1,angle);
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
    p_CommonData->p_tissueTwo->setStiffness(tissueNomStiffness*1.125, true);
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
    p_CommonData->p_tissueFour->setStiffness(tissueNomStiffness*1.375, true);
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
    p_CommonData->p_tissueFive->setStiffness(tissueNomStiffness*1.5, true);
    p_CommonData->p_tissueFive->setFriction(staticFriction, dynamicFriction, TRUE);

    //----------------------------------------------Create Tissue Indicator---------------------------------------------------
    // add object to world
    world->addChild(p_CommonData->p_tissueSix);

    p_CommonData->p_tissueSix->setLocalPos(0,0,-.026+vertOffset);

    //load the object from file
    p_CommonData->p_tissueSix->loadFromFile("./Resources/tissue_5 OBJ/tissue_5.obj");

    p_CommonData->p_tissueSix->setHapticEnabled(false);
    chai3d::cMaterial mat0;
    mat0.setRed();
    p_CommonData->p_tissueSix->setMaterial(mat0);
    p_CommonData->p_tissueSix->setTransparencyLevel(0);

}




