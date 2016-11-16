#include "haptics_thread.h"

haptics_thread::haptics_thread(QObject *parent) : QThread(parent)
{

}

haptics_thread::~haptics_thread()
{

}

void haptics_thread::initialize()
{
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
    p_CommonData->deviceComputedForce.set(0,0,0);
    p_CommonData->filteredDeviceComputedForce.set(0,0,0);

    //p_CommonData->workspaceScaleFactor = 1;

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

    p_CommonData->fingerDisplayScale = 1;
    p_CommonData->box1displayScale = 1;
    p_CommonData->expCD = 1.0;

    p_CommonData->fingerScalePoint.set(0,0,0);

    fingerOffset.set(0,-0.006,.003); // finger axis are not at fingerpad, so we want a translation outward on fingertip
    thumbOffset.set(0,-0.009,.003); // finger axis are not at fingerpad, so we want a translation outward on fingertip

    // initial box positions
    p_CommonData->box1InitPos.set(0,  0,  -0.030);
    p_CommonData->box2InitPos.set(1,   0,  0.0);
    p_CommonData->box3InitPos.set(1, -.1,  0.0);

    p_CommonData->box1PostInitCenter.set(0,0,-0.025);

    p_CommonData->fingerTouching = false; //reset before we check
    p_CommonData->thumbTouching = false;
    p_CommonData->fingerTouchingLast = false;
    p_CommonData->thumbTouchingLast = false;
    p_CommonData->scaledDispTransp = 1;
    p_CommonData->clutchedOffset.set(0,0,0);
    p_CommonData->fingerDisplayScale = 1.0; //will get changed in dynsim if necessary

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

            Eigen::Vector3d inputAxis(0,1,0); // input axis for sin control and circ control modes
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
                p_CommonData->sharedMutex.lock();
                UpdateVRGraphics();                
                p_CommonData->sharedMutex.unlock();
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
            if(recordDataCounter == 15)
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
#ifndef OCULUS
    // Update camera Pos
    double xPos = p_CommonData->camRadius*cos(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double yPos = p_CommonData->camRadius*sin(p_CommonData->azimuth*PI/180.0)*sin(p_CommonData->polar*PI/180.0);
    double zPos = p_CommonData->camRadius*cos(p_CommonData->polar*PI/180.0);
    p_CommonData->cameraPos.set(xPos, yPos, zPos);
#endif

#ifdef OCULUS
    p_CommonData->lookatPos.set(0, 0, p_CommonData->cameraPos.z());
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
            p_CommonData->p_world->clearAllChildren();
            p_CommonData->p_world->addChild(m_tool0);
            p_CommonData->p_world->addChild(m_tool1);
            p_CommonData->p_world->addChild(finger);
            p_CommonData->p_world->addChild(thumb);
            break;

        case experimentFriction:
            p_CommonData->p_world->clearAllChildren();
            RenderExpFriction();
            break;

        case twoFriction:
            p_CommonData->p_world->clearAllChildren();
            RenderTwoFriction();
            break;

        case palpation:
            p_CommonData->p_world->clearAllChildren();
            RenderPalpation();
            break;

        case experimentPalpationLine:
            p_CommonData->p_world->clearAllChildren();
            RenderPalpation();
            break;

        case dynamicBodies:
            p_CommonData->p_world->clearAllChildren();
            RenderDynamicBodies();
            break;
        }
    }

    // compute global reference frames for each object
    p_CommonData->p_world->computeGlobalPositions(true);

    // update position and orientation of tool 0(and sphere that represents tool)
    m_tool0->updateFromDevice();
    position0 = m_tool0->m_hapticPoint->getGlobalPosGoal(); // get position and rotation of the haptic point (and delta mechanism) (already transformed from magTracker)
    p_CommonData->chaiMagDevice0->getRotation(rotation0);
    deviceRotation0 = rotation0; //the chai device already rotates the tracker return based on the finger harness

    // update position and orientation of tool 1
    m_tool1->updateFromDevice();
    position1 = m_tool1->m_hapticPoint->getGlobalPosGoal();
    p_CommonData->chaiMagDevice1->getRotation(rotation1);
    deviceRotation1 = rotation1;

    // update position of finger to stay on proxy point
    fingerRotation0 = rotation0;
    fingerRotation0.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation0.rotateAboutLocalAxisDeg(1,0,0,90);
    finger->setLocalRot(fingerRotation0);
    finger->setLocalPos(m_tool0->m_hapticPoint->getGlobalPosProxy() + fingerRotation0*fingerOffset); //this offset isn't for computation of forces, just to align finger model
    m_curSphere0->setLocalPos(position0); // set the sphere visual representation to match
    m_curSphere0->setLocalRot(rotation0);
    m_tool0->computeInteractionForces();

    // update position of finger to stay on proxy  point
    fingerRotation1 = rotation1;
    fingerRotation1.rotateAboutLocalAxisDeg(0,0,1,90);
    fingerRotation1.rotateAboutLocalAxisDeg(1,0,0,90);
    thumb->setLocalRot(fingerRotation1);
    thumb->setLocalPos(m_tool1->m_hapticPoint->getGlobalPosProxy() + fingerRotation1*thumbOffset);
    m_curSphere1->setLocalPos(position1);
    m_curSphere1->setLocalRot(rotation1);
    m_tool1->computeInteractionForces();

    //check for applying scaling based on whether we were touching last time
    p_CommonData->fingerTouchingLast = p_CommonData->fingerTouching;
    p_CommonData->thumbTouchingLast = p_CommonData->thumbTouching;

    p_CommonData->fingerTouching = false; //reset before we check
    p_CommonData->thumbTouching = false;

    currTime = p_CommonData->overallClock.getCurrentTimeSeconds();
    timeInterval = currTime - lastTime;
    if(timeInterval > .001)
        timeInterval = 0.001;

    // perform our dynamic body updates if we are in a dynamic environment
    if(p_CommonData->currentEnvironmentState == dynamicBodies)
    {
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

                //if finger touching box, note it
                p_CommonData->fingerTouching = true;

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    if(!(interactionPoint->getLastComputedForce().length() > 40))
                        ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                                                           collisionEvent->m_globalPos);
                    else
                    {
                        p_CommonData->resetBoxPosFlag = true;
                    }
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

                //if thumb touching box, note it
                p_CommonData->thumbTouching = true;

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    if(!(interactionPoint->getLastComputedForce().length() > 40))
                        ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                                                           collisionEvent->m_globalPos);
                    else
                    {
                        p_CommonData->resetBoxPosFlag = true;
                    }
                }
            }
        }

        // update simulation
        ODEWorld->updateDynamics(timeInterval);

        // update display scaling and scaling center if we just grabbed a box or are pushing and need to scale and center
        if (p_CommonData->fingerTouching | p_CommonData->thumbTouching)
        {
            // need this to only run the first time they're both touching
            if(!p_CommonData->fingerTouchingLast & !p_CommonData->thumbTouchingLast)
            {
                p_CommonData->fingerScalePoint = curCenter;
                p_CommonData->fingerDisplayScale = 1.0/p_CommonData->expCD;
            }
        }

        // update display scaling center if we just dropped a box
        if (!p_CommonData->fingerTouching & !p_CommonData->thumbTouching)
        {
            if (p_CommonData->fingerTouchingLast | p_CommonData->thumbTouchingLast)
            {
                p_CommonData->clutchedOffset = (scaledCurCenter - curCenter);
                p_CommonData->fingerDisplayScale = 1.0;
            }
        }
    }        
    lastTime = currTime;

    // update scaled positions
    UpdateScaledCursors();
    UpdateScaledFingers();
    UpdateScaledBoxes();

    // move the box, but after we've already reset the fingers
    if(p_CommonData->resetBoxPosFlag)
    {
        // set position of box back to starting point
        chai3d::cMatrix3d eyeMat(1,0,0,0,1,0,0,0,1);
        p_CommonData->ODEBody1->setLocalPos(0,0,-1);
        p_CommonData->ODEBody1->setLocalRot(eyeMat);
        ODEWorld->updateDynamics(timeInterval);

        m_tool0->updateFromDevice();
        m_tool0->computeInteractionForces();
        m_tool1->updateFromDevice();
        m_tool1->computeInteractionForces();
        ODEWorld->updateDynamics(timeInterval);

        p_CommonData->ODEBody1->setLocalPos(p_CommonData->box1InitPos);
        p_CommonData->ODEBody1->setLocalRot(eyeMat);
        ODEWorld->updateDynamics(timeInterval);

        p_CommonData->resetBoxPosFlag = false;
    }

    // set scaled stuff to show or not show
    UpdateScaledTransparency();

    // Remove environment if experiments done
    if(p_CommonData->expDone)
    {
        p_CommonData->environmentChange = true;
        p_CommonData->currentEnvironmentState = none;
        p_CommonData->expDone = false;
    }
}

void haptics_thread::UpdateScaledTransparency()
{
    // case invisible scaled display
    if ((p_CommonData->scaledDispTransp % 3) == 0)
    {
        scaledFinger->setTransparencyLevel(0.0);
        scaledThumb->setTransparencyLevel(0.0);
        p_CommonData->p_dynamicScaledBox1->setTransparencyLevel(0.0);

        finger->setTransparencyLevel(1.0);
        thumb->setTransparencyLevel(1.0);
        p_CommonData->p_dynamicBox1->setTransparencyLevel(1.0);
    }
    if ((p_CommonData->scaledDispTransp % 3) == 1)
    {
        scaledFinger->setTransparencyLevel(1.0);
        scaledThumb->setTransparencyLevel(1.0);
        p_CommonData->p_dynamicScaledBox1->setTransparencyLevel(1.0);

        finger->setTransparencyLevel(0);
        thumb->setTransparencyLevel(0);
        p_CommonData->p_dynamicBox1->setTransparencyLevel(0);
    }
    if ((p_CommonData->scaledDispTransp % 3) == 2)
    {
        scaledFinger->setTransparencyLevel(1);
        scaledThumb->setTransparencyLevel(1);
        p_CommonData->p_dynamicScaledBox1->setTransparencyLevel(1);

        finger->setTransparencyLevel(0.5);
        thumb->setTransparencyLevel(0.5);
        p_CommonData->p_dynamicBox1->setTransparencyLevel(1);
    }
}

void haptics_thread::UpdateScaledCursors()
{
    curCenter = (m_tool0->m_hapticPoint->getGlobalPosProxy() + m_tool1->m_hapticPoint->getGlobalPosProxy())/2.0;
    centToFingCur = m_tool0->m_hapticPoint->getGlobalPosProxy() - curCenter;
    centToThumbCur = m_tool1->m_hapticPoint->getGlobalPosProxy() - curCenter;

    scaledCurCenter = p_CommonData->fingerDisplayScale*(curCenter - p_CommonData->fingerScalePoint) + p_CommonData->fingerScalePoint + p_CommonData->clutchedOffset;
    m_dispScaleCurSphere0->setLocalPos(scaledCurCenter + centToFingCur);
    m_dispScaleCurSphere1->setLocalPos(scaledCurCenter + centToThumbCur);
}

void haptics_thread::UpdateScaledFingers()
{
    // update position of finger to stay on scaled cursor (which is scaled relative to proxy point)
    scaledFinger->setLocalRot(fingerRotation0);
    scaledFinger->setLocalPos(m_dispScaleCurSphere0->getLocalPos() + fingerRotation0*fingerOffset);

    // update position of thumb to stay on scaled cursor (which is scaled relative to proxy point)
    scaledThumb->setLocalRot(fingerRotation1);
    scaledThumb->setLocalPos(m_dispScaleCurSphere1->getLocalPos() + fingerRotation1*thumbOffset);
 }

void haptics_thread::UpdateScaledBoxes()
{
    p_CommonData->box1displayScale = 1.0/p_CommonData->expCD;

    chai3d::cVector3d scaledBox1Pos; scaledBox1Pos = p_CommonData->box1PostInitCenter + (p_CommonData->ODEBody1->getLocalPos()-p_CommonData->box1PostInitCenter)*p_CommonData->box1displayScale;

    p_CommonData->p_dynamicScaledBox1->setLocalPos(scaledBox1Pos);
    p_CommonData->p_dynamicScaledBox1->setLocalRot(p_CommonData->ODEBody1->getLocalRot());
}

void haptics_thread::ComputeVRDesiredDevicePos()
{
    // perform transformation to get "device forces"
    computedForce0 = m_tool0->getDeviceGlobalForce();
    computedForce1 = m_tool1->getDeviceGlobalForce();

    if(computedForce0.length() > 40)
        computedForce0.set(0,0,0);
    if(computedForce1.length() > 40)
        computedForce1.set(0,0,0);

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
    double fc = 5.0;
    double RC = 1.0/(fc*2.0*PI);
    double alpha = (timeInterval)/(RC + timeInterval);

    // get filtered force
    filteredDeviceForce0 = alpha*deviceComputedForce0 + (1-alpha)*lastFilteredDeviceForce0;
    filteredDeviceForce1 = alpha*deviceComputedForce1 + (1-alpha)*lastFilteredDeviceForce1;    

//    // assign to the shared data structure to allow plotting from window thread
//    p_CommonData->deviceComputedForce = deviceComputedForce0;
//    p_CommonData->filteredDeviceComputedForce = filteredDeviceForce0;

    // add impulses here so as not to affect filter
    AddImpulseDisp(indexImpulse, thumbImpulse);
    AddImpulseTorqueDisp(indexTorqueImpulse, thumbTorqueImpulse);

    // rotate impulse into device frames
    indexImpulse = deviceRotation0*rotation0*indexImpulse;
    thumbImpulse = deviceRotation1*rotation1*thumbImpulse;
    indexTorqueImpulse = deviceRotation0*rotation0*indexTorqueImpulse;
    thumbTorqueImpulse = deviceRotation1*rotation1*thumbTorqueImpulse;

    //convert device "force" to a mapped position
    double forceToPosMult = 1.0/1.588; // based on lateral stiffness of finger (averaged directions from Gleeson paper) (1.588 N/mm)
    double forceToPosMultThumb = forceToPosMult;

//    double forceToPosMult = 1.0/0.5; // based on lateral shearing stiffness of finger (from new Nakazawa paper, Srini paper supports similar for normal force) (0.5 N/mm)
//    double forceToPosMultThumb = forceToPosMult;

    // Pos movements in delta mechanism frame (index)
    chai3d::cVector3d desiredPosMovement0 = forceToPosMult*(filteredDeviceForce0 + indexImpulse + indexTorqueImpulse); //this is only for lateral if we override normal later
    chai3d::cVector3d desiredPosMovement1 = forceToPosMultThumb*(filteredDeviceForce1 + thumbImpulse + thumbTorqueImpulse); //this is only for lateral if we override normal later

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

    // don't allow the tactor to move away from finger when computing VR interaction
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
    p_CommonData->dataRecordMutex.lock();

    // modified for new experiment, now with both devices
    recordDataCounter = 0;
    p_CommonData->dataRecorder.time = p_CommonData->overallClock.getCurrentTimeSeconds();
    p_CommonData->dataRecorder.jointAngles0 = p_CommonData->wearableDelta0->GetJointAngles();
    p_CommonData->dataRecorder.motorAngles0 = p_CommonData->wearableDelta0->GetMotorAngles();
    p_CommonData->dataRecorder.pos0 = p_CommonData->wearableDelta0->GetCartesianPos();
    p_CommonData->dataRecorder.desiredPos0 = p_CommonData->wearableDelta0->ReadDesiredPos();
    p_CommonData->dataRecorder.voltageOut0 = p_CommonData->wearableDelta0->ReadVoltageOutput();
    p_CommonData->dataRecorder.VRInteractionForce0 = deviceForceRecord0; // last force on tool0
    p_CommonData->dataRecorder.VRInteractionForceGlobal0 = globalForceRecord0; // last force on tool0 in global coords
    p_CommonData->dataRecorder.motorTorque0 = p_CommonData->wearableDelta0->motorTorques;
    p_CommonData->dataRecorder.magTrackerPos0 = position0;

    p_CommonData->dataRecorder.jointAngles1 = p_CommonData->wearableDelta1->GetJointAngles();
    p_CommonData->dataRecorder.motorAngles1 = p_CommonData->wearableDelta1->GetMotorAngles();
    p_CommonData->dataRecorder.pos1 = p_CommonData->wearableDelta1->GetCartesianPos();
    p_CommonData->dataRecorder.desiredPos1 = p_CommonData->wearableDelta1->ReadDesiredPos();
    p_CommonData->dataRecorder.voltageOut1 = p_CommonData->wearableDelta1->ReadVoltageOutput();
    p_CommonData->dataRecorder.VRInteractionForce1 = deviceForceRecord1; // last force on tool0
    p_CommonData->dataRecorder.VRInteractionForceGlobal1 = globalForceRecord1; // last force on tool0 in global coords
    p_CommonData->dataRecorder.motorTorque1 = p_CommonData->wearableDelta1->motorTorques;
    p_CommonData->dataRecorder.magTrackerPos1 = position1;

    p_CommonData->dataRecorder.deviceRotation0 = deviceRotation0*rotation0;
    p_CommonData->dataRecorder.deviceRotation1 = deviceRotation1*rotation1;

    p_CommonData->dataRecorder.box1Pos = p_CommonData->ODEBody1->getLocalPos();
    p_CommonData->dataRecorder.scaledBox1Pos = p_CommonData->p_dynamicScaledBox1->getLocalPos();

    p_CommonData->dataRecorder.pairNo = p_CommonData->pairNo;
    p_CommonData->dataRecorder.boxMass = p_CommonData->expMass;
    p_CommonData->dataRecorder.CDRatio = p_CommonData->expCD;
    p_CommonData->dataRecorder.isRef = p_CommonData->isRef;
    p_CommonData->dataRecorder.subjResponse = 0;//fill in right before recording in window thread
    p_CommonData->dataRecorder.isUpperCurve = p_CommonData->isUpperCurve;

    //p_CommonData->dataRecorder.isReversal handled by mainwindow thread manually at end of each trial

    p_CommonData->dataRecorderVector.push_back(p_CommonData->dataRecorder);

    p_CommonData->dataRecordMutex.unlock();
}

void haptics_thread::InitGeneralChaiStuff()
{
    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------
    // Create a new world
    p_CommonData->p_world = new chai3d::cWorld();

    // create a camera and insert it into the virtual world
    p_CommonData->p_world->setBackgroundColor(0, 0, 0);
    p_CommonData->p_world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    p_CommonData->p_camera = new chai3d::cCamera(p_CommonData->p_world);
    p_CommonData->p_world->addChild(p_CommonData->p_camera);

    // Position and orientate the camera
    // X is toward camera, pos y is to right, pos z is up
    p_CommonData->cameraPos.set(0.320, 0.0, -.350);
    p_CommonData->lookatPos.set(0.0, 0.0, 0.0);
    p_CommonData->upVector.set(0.0, 0.0, -1.0);
    p_CommonData->p_camera->set( p_CommonData->cameraPos,    //(0.25, 0, -.25),    // camera position (eye)
                                 p_CommonData->lookatPos,    // lookat position (target)
                                 p_CommonData->upVector);    // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    p_CommonData->p_camera->setClippingPlanes(0.01, 20.0);

#ifndef OCULUS
    // the camera is updated to a position based on these params
    p_CommonData->azimuth = 0.0;
    p_CommonData->polar = 135.0; //higher number puts us higher in the air
    p_CommonData->camRadius = 0.45;
#endif

    // create a light source and attach it to the camera
    light = new chai3d::cSpotLight(p_CommonData->p_world);
    p_CommonData->p_world->addChild(light);   // insert light source inside world
    light->setEnabled(true);                   // enable light source
    light->setDir(-.2, -0.2, .5);  // define the direction of the light beam
    light->setLocalPos(.2, 0.2, -.5);
    light->setCutOffAngleDeg(120);
    light->setShadowMapEnabled(true);
}

void haptics_thread::InitFingerAndTool()
{
    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
    m_tool0 = new chai3d::cToolCursor(p_CommonData->p_world); // create a 3D tool
    p_CommonData->p_world->addChild(m_tool0); //insert the tool into the world
    toolRadius = 0.002; // set tool radius
    m_tool0->setRadius(toolRadius);
    m_tool0->setHapticDevice(p_CommonData->chaiMagDevice0); // connect the haptic device to the tool
    m_tool0->setShowContactPoints(false, false, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool0->enableDynamicObjects(true);
    m_tool0->setWaitForSmallForce(true);
    m_tool0->start();

    //uncomment this if we want to use 2 tools
    m_tool1 = new chai3d::cToolCursor(p_CommonData->p_world); // create a 3D tool
    p_CommonData->p_world->addChild(m_tool1); //insert the tool into the world
    m_tool1->setRadius(toolRadius);
    m_tool1->setHapticDevice(p_CommonData->chaiMagDevice1); // connect the haptic device to the tool
    m_tool1->setShowContactPoints(false, false, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool1->enableDynamicObjects(true);
    m_tool1->setWaitForSmallForce(true);
    m_tool1->start();

    // Can use this to show frames on tool if so desired
    //create a sphere to represent the tool
    m_curSphere0 = new chai3d::cShapeSphere(toolRadius);
    p_CommonData->p_world->addChild(m_curSphere0);
    m_curSphere0->m_material->setGrayDarkSlate();
    m_curSphere0->setShowFrame(true);
    m_curSphere0->setFrameSize(0.05);

    m_curSphere1 = new chai3d::cShapeSphere(toolRadius);
    p_CommonData->p_world->addChild(m_curSphere1);
    m_curSphere1->m_material->setBlueAqua();
    m_curSphere1->setShowFrame(true);
    m_curSphere1->setFrameSize(0.05);

    // add display scaled visual representations
    m_dispScaleCurSphere0 = new chai3d::cShapeSphere(toolRadius);
    p_CommonData->p_world->addChild(m_dispScaleCurSphere0);
    m_dispScaleCurSphere0->m_material->setGrayDarkSlate();
    m_dispScaleCurSphere0->setShowFrame(false);
    m_dispScaleCurSphere0->setFrameSize(0.05);

    m_dispScaleCurSphere1 = new chai3d::cShapeSphere(toolRadius);
    p_CommonData->p_world->addChild(m_dispScaleCurSphere1);
    m_dispScaleCurSphere1->m_material->setBlueAqua();
    m_dispScaleCurSphere1->setShowFrame(false);
    m_dispScaleCurSphere1->setFrameSize(0.05);


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------
    // create a finger object
    finger = new chai3d::cMultiMesh(); // create a virtual mesh
    p_CommonData->p_world->addChild(finger); // add object to world
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0.0, 0.0, 0.0);

    thumb = new chai3d::cMultiMesh(); //create the thumb
    p_CommonData->p_world->addChild(thumb);
    thumb->setShowFrame(false);
    thumb->setFrameSize(0.05);
    thumb->setLocalPos(0,0,0);

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

    ////////////////////////////////////////////
    // CREATE POSITION SCALED OBJECTS
    /////////////////////////////////////////////
    scaledFinger = new chai3d::cMultiMesh(); // create a virtual mesh
    p_CommonData->p_world->addChild(scaledFinger); // add object to world
    scaledFinger->setShowFrame(false);
    scaledFinger->setFrameSize(0.05);
    scaledFinger->setLocalPos(0.0, 0.0, 0.0);

    scaledThumb = new chai3d::cMultiMesh(); //create the scaledThumb
    p_CommonData->p_world->addChild(scaledThumb);
    scaledThumb->setShowFrame(false);
    scaledThumb->setFrameSize(0.05);
    scaledThumb->setLocalPos(0,0,0);

    if(cLoadFileOBJ(scaledFinger, "./Resources/FingerModel.obj")){
        qDebug() << "finger file loaded";
    }
    if(cLoadFileOBJ(scaledThumb, "./Resources/FingerModelThumb.obj")){
        qDebug() << "thumb file loaded";
    }

    // set params for scaledFinger
    scaledFinger->setShowEnabled(true);
    scaledFinger->setUseVertexColors(true);
    chai3d::cColorf scaledFingerColor;
    scaledFingerColor.setBrownSandy();
    scaledFinger->setVertexColor(scaledFingerColor);
    scaledFinger->m_material->m_ambient.set(0.1, 0.1, 0.1);
    scaledFinger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    scaledFinger->m_material->m_specular.set(1.0, 1.0, 1.0);
    scaledFinger->setUseMaterial(true);
    scaledFinger->setHapticEnabled(false);

    // set params for scaledThumb
    scaledThumb->setShowEnabled(true);
    scaledThumb->setUseVertexColors(true);
    chai3d::cColorf scaledThumbColor;
    scaledThumbColor.setBrownSandy();
    scaledThumb->setVertexColor(scaledThumbColor);
    scaledThumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
    scaledThumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    scaledThumb->m_material->m_specular.set(1.0, 1.0, 1.0);
    scaledThumb->setUseMaterial(true);
    scaledThumb->setHapticEnabled(false);

}

void haptics_thread::InitEnvironments()
{
    p_CommonData->p_frictionBox1 = new chai3d::cMesh();
    p_CommonData->p_frictionBox2 = new chai3d::cMesh();
    p_CommonData->p_textureBox = new chai3d::cMultiMesh();
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

    // Create the ODE objects for the blocks and cup
    p_CommonData->ODEBody1 = new cODEGenericBody(ODEWorld);
//    p_CommonData->ODEBody2 = new cODEGenericBody(ODEWorld);
//    p_CommonData->ODEBody3 = new cODEGenericBody(ODEWorld);
//    p_CommonData->ODEBody4 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh that will be used for the geometry representation of the dynamic body
    p_CommonData->p_dynamicBox1 = new chai3d::cMesh();
//    p_CommonData->p_dynamicBox2 = new chai3d::cMesh();
//    p_CommonData->p_dynamicBox3 = new chai3d::cMesh();
//    p_CommonData->p_dynamicBox4 = new chai3d::cMesh();

    // create the scaled virtual objects
    p_CommonData->p_dynamicScaledBox1 = new chai3d::cMesh();
//    p_CommonData->p_dynamicScaledBox2 = new chai3d::cMesh();
//    p_CommonData->p_dynamicScaledBox3 = new chai3d::cMesh();
//    p_CommonData->p_dynamicScaledBox4 = new chai3d::cMesh();

    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
//    ODEGPlane1 = new cODEGenericBody(ODEWorld);

    //create ground
    ground = new chai3d::cMesh();

    //create background mesh
    globe = new chai3d::cMesh();

    ////////////////////////////////////////////
    // CREATE 1 and 2 MODELS FOR EXPERIMENT DISPLAY
    /////////////////////////////////////////////
    //init one and two meshes
    p_CommonData->oneModel = new chai3d::cMultiMesh();
    p_CommonData->twoModel = new chai3d::cMultiMesh();
    p_CommonData->p_world->addChild(p_CommonData->oneModel); // add object to world
    p_CommonData->p_world->addChild(p_CommonData->twoModel); // add object to world

    // load an object file
    if(cLoadFileOBJ(p_CommonData->oneModel, "./Resources/One.obj")){
        qDebug() << "'One' loaded";
    }
    if(cLoadFileOBJ(p_CommonData->twoModel, "./Resources/Two.obj")){
        qDebug() << "'Two' loaded";
    }

    p_CommonData->oneModel->setShowEnabled(true);
    p_CommonData->oneModel->setUseVertexColors(true);
    chai3d::cColorf oneColor;
    oneColor.setRedCrimson();
    p_CommonData->oneModel->setVertexColor(oneColor);

    p_CommonData->twoModel->setShowEnabled(true);
    p_CommonData->twoModel->setUseVertexColors(true);
    chai3d::cColorf twoColor;
    twoColor.setRedCrimson();
    p_CommonData->twoModel->setVertexColor(twoColor);

    p_CommonData->oneModel->rotateAboutLocalAxisDeg(chai3d::cVector3d(0,0,1), -90);
    p_CommonData->twoModel->rotateAboutLocalAxisDeg(chai3d::cVector3d(0,0,1), -90);
    p_CommonData->oneModel->rotateAboutLocalAxisDeg(chai3d::cVector3d(1,0,0), -90);
    p_CommonData->twoModel->rotateAboutLocalAxisDeg(chai3d::cVector3d(1,0,0), -90);

}
void haptics_thread::DeleteDynamicBodies()
{
    delete ODEWorld;
    delete p_CommonData->oneModel;
    delete p_CommonData->twoModel;
    delete p_CommonData->ODEBody1;
    delete p_CommonData->ODEBody2;
    delete p_CommonData->ODEBody3;
    delete p_CommonData->ODEBody4;
    delete p_CommonData->p_dynamicBox1;
    delete p_CommonData->p_dynamicScaledBox1;
    delete p_CommonData->p_dynamicBox2;
    delete p_CommonData->p_dynamicBox3;
    delete p_CommonData->p_dynamicBox4;
    delete ODEGPlane0;
    delete ground;
    delete globe;

    p_CommonData->p_world->removeChild(p_CommonData->p_dynamicBox1);
    p_CommonData->p_world->removeChild(p_CommonData->p_dynamicScaledBox1);
    p_CommonData->p_world->removeChild(ODEWorld);
    p_CommonData->p_world->removeChild(ground);
    p_CommonData->p_world->removeChild(m_tool0);
    p_CommonData->p_world->removeChild(m_tool1);
    p_CommonData->p_world->removeChild(finger);
    p_CommonData->p_world->removeChild(thumb);
    p_CommonData->p_world->removeChild(globe);

    // add scaled bodies for altering display ratio
    p_CommonData->p_world->removeChild(m_dispScaleCurSphere0);
    p_CommonData->p_world->removeChild(m_dispScaleCurSphere1);
    p_CommonData->p_world->removeChild(scaledFinger);
    p_CommonData->p_world->removeChild(scaledThumb);

}

void haptics_thread::RenderDynamicBodies()
{
    DeleteDynamicBodies();
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
    ODEWorld->setAngularDamping(.02);
    ODEWorld->setLinearDamping(.005);

    //create a plane
    groundSize = .3;
    groundThickness = 0.01;
    //--------------------------------------------------------------------------
    // CREATING ODE INVISIBLE WALLS
    //--------------------------------------------------------------------------
    ODEGPlane0->createStaticPlane(chai3d::cVector3d(0.0, 0.0, 0), chai3d::cVector3d(0.0, 0.0 ,-1.0));

    chai3d::cCreateBox(ground, groundSize, 2*groundSize, groundThickness);
    ground->setLocalPos(0,0,groundThickness*0.5);

    //create globe
    chai3d::cCreateSphere(globe, 10, 30, 30);
    globe->setUseDisplayList(true);
    globe->deleteCollisionDetector();

    // create a texture
    textureSpace = chai3d::cTexture2d::create();
    textureSpace->loadFromFile("./Resources/sky.jpg");

    globe->setTexture(textureSpace, true); //apply texture to globe
    globe->setUseCulling(false, true);     // Since we don't need to see our polygons from both sides, we enable culling.

    // define some material properties for ground
    chai3d::cMaterial matGround;
    matGround.setBrownSandy();
    ground->setMaterial(matGround);

    // choose which type of dynamic object environ to render
    switch(p_CommonData->currentDynamicObjectState)
    {
    case standard:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = .15; mass2 = 0.15; mass3 = 0.15;
        stiffness1 = 500; stiffness2 = 500; stiffness3 = 500;
        break;
    case mass:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = .05; mass2 = 0.2; mass3 = 0.25;
        stiffness1 = 500; stiffness2 = 500; stiffness3 = 500;
        break;
    case friction:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 0.5; friction2 = 2.0; friction3 = 4.0;
        mass1 = 0.15; mass2 = 0.15; mass3 = 0.15;
        stiffness1 = 500; stiffness2 = 500; stiffness3 = 500;
        break;
    case dimension:
        boxSize1 = 0.03; boxSize2 = 0.045; boxSize3 = 0.06;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = .1; mass2 = 0.1; mass3 = 0.1;
        stiffness1 = 500; stiffness2 = 500; stiffness3 = 500;
        break;
    case stiffness:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = .15; mass2 = 0.15; mass3 = 0.15;
        stiffness1 = 200; stiffness2 = 400; stiffness3 = 600;
        break;
    case dynamicExperiment:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = 0.2; mass2 = 0.2; mass3 = 0.2;
        stiffness1 = 500; stiffness2 = 500; stiffness3 = 500;
        break;
    case dynamicCDExp:
        boxSize1 = 0.05; boxSize2 = 0.05; boxSize3 = 0.05;
        friction1 = 2.0; friction2 = 2.0; friction3 = 2.0;
        mass1 = 0.2; mass2 = 0.2; mass3 = 0.2;
        stiffness1 = 450; stiffness2 = 450; stiffness3 = 450;
        break;
    }

    //assign the params dependent on the others
    latStiffness1 = stiffness1*1.5; latStiffness2 = stiffness2*1.5; latStiffness3 = stiffness3*1.5;
    dynFriction1 = 0.9*friction1; dynFriction2 = 0.9*friction2; dynFriction3 = 0.9*friction3;

    if (p_CommonData->currentDynamicObjectState == dynamicExperiment)
    {
        SetDynEnvironMassExp();
    }

    else if (p_CommonData->currentDynamicObjectState == dynamicCDExp)
    {
        SetDynEnvironCDExp();
    }

    // if just rendering dynamic environments without an experiment
    else
    {
        SetDynEnvironDemo();
    }

    //set position of backgroundObject
    globe->setLocalPos(0,0,0);

    // setup tools for dynamic interaction
    m_tool0->enableDynamicObjects(true);
    m_tool1->enableDynamicObjects(true);

    //add objects to world
    p_CommonData->p_world->addChild(ODEWorld);
    p_CommonData->p_world->addChild(ground);
    p_CommonData->p_world->addChild(m_tool0);
    p_CommonData->p_world->addChild(m_tool1);
    p_CommonData->p_world->addChild(finger);
    p_CommonData->p_world->addChild(thumb);
    p_CommonData->p_world->addChild(globe);

    // add scaled bodies for altering display ratio
    m_dispScaleCurSphere0->setHapticEnabled(false);
    m_dispScaleCurSphere1->setHapticEnabled(false);
    scaledFinger->setHapticEnabled(false);
    scaledThumb->setHapticEnabled(false);
    p_CommonData->p_world->addChild(m_dispScaleCurSphere0);
    p_CommonData->p_world->addChild(m_dispScaleCurSphere1);
    p_CommonData->p_world->addChild(scaledFinger);
    p_CommonData->p_world->addChild(scaledThumb);

    p_CommonData->clutchedOffset.set(0,0,0);
    p_CommonData->fingerScalePoint.set(0,0,0);

    m_tool0->setTransparencyLevel(0);
    m_tool1->setTransparencyLevel(0);
    m_curSphere0->setTransparencyLevel(0);
    m_curSphere1->setTransparencyLevel(0);
    m_dispScaleCurSphere0->setTransparencyLevel(0);
    m_dispScaleCurSphere1->setTransparencyLevel(0);

}

void haptics_thread::SetDynEnvironCDExp()
{
    // create the visual boxes on the dynamicbox meshes
    cCreateBox(p_CommonData->p_dynamicBox1, boxSize1, boxSize1, boxSize1); // make mesh a box

    // create the visual for the position scaled dynamic boxes
    cCreateBox(p_CommonData->p_dynamicScaledBox1, boxSize1, boxSize1, boxSize1); // make mesh a box

    // setup collision detectorsfor the dynamic objects
    p_CommonData->p_dynamicBox1->createAABBCollisionDetector(toolRadius);

    // define material properties for box 1
    chai3d::cMaterial mat1;
    mat1.setRedCrimson();
    mat1.setStiffness(stiffness1);
    mat1.setLateralStiffness(latStiffness1);
    mat1.setDynamicFriction(dynFriction1);
    mat1.setStaticFriction(friction1);
    mat1.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox1->setMaterial(mat1);
    p_CommonData->p_dynamicBox1->setUseMaterial(true);

    // define material properties for box 1
    chai3d::cMaterial mat2;
    mat2.setBlueRoyal();
    p_CommonData->p_dynamicScaledBox1->setMaterial(mat2);
    p_CommonData->p_dynamicScaledBox1->setUseMaterial(true);
    p_CommonData->p_dynamicScaledBox1->setHapticEnabled(false);

    // add mesh to ODE object
    p_CommonData->ODEBody1->setImageModel(p_CommonData->p_dynamicBox1);

    // create a dynamic model of the ODE object
    p_CommonData->ODEBody1->createDynamicBox(boxSize1, boxSize1, boxSize1);

    // set mass of box
    p_CommonData->ODEBody1->setMass(p_CommonData->expMass);

    // set position of box
    p_CommonData->ODEBody1->setLocalPos(p_CommonData->box1InitPos);

    //add one and two indicators
    p_CommonData->p_world->addChild(p_CommonData->oneModel);
    p_CommonData->p_world->addChild(p_CommonData->twoModel);

    p_CommonData->p_world->addChild(p_CommonData->p_dynamicBox1);
    p_CommonData->p_world->addChild(p_CommonData->p_dynamicScaledBox1);

    p_CommonData->oneModel->setLocalPos(0.07, 0.12, 0);
    p_CommonData->twoModel->setLocalPos(0.07, -0.10, 0);
    p_CommonData->oneModel->setTransparencyLevel(1.0);
    p_CommonData->twoModel->setTransparencyLevel(0.0);
}

// was for sizeWeight CHI exp
void haptics_thread::SetDynEnvironMassExp()
{
    cCreateBox(p_CommonData->p_dynamicBox1, boxSize1, boxSize1, boxSize1); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox2, boxSize2, boxSize2, 2*boxSize2); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox3, boxSize3, boxSize3, 3*boxSize3); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox4, boxSize3, boxSize3, 2*boxSize3); // make mesh a box
    p_CommonData->p_dynamicBox1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox2->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox3->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox4->createAABBCollisionDetector(toolRadius);
    // define material properties for box 1
    chai3d::cMaterial mat1;
    mat1.setRedCrimson();
    mat1.setStiffness(stiffness1);
    mat1.setLateralStiffness(latStiffness1);
    mat1.setDynamicFriction(dynFriction1);
    mat1.setStaticFriction(friction1);
    mat1.setUseHapticFriction(true);
    // define material properties for box 2
    chai3d::cMaterial mat2;
    mat2.setBlueRoyal();
    mat2.setStiffness(stiffness2);
    mat2.setLateralStiffness(latStiffness2);
    mat2.setDynamicFriction(dynFriction2);
    mat2.setStaticFriction(friction2);
    mat2.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox1->setMaterial(mat1);
    p_CommonData->p_dynamicBox1->setUseMaterial(true);
    p_CommonData->p_dynamicBox2->setMaterial(mat1);
    p_CommonData->p_dynamicBox2->setUseMaterial(true);
    p_CommonData->p_dynamicBox3->setMaterial(mat1);
    p_CommonData->p_dynamicBox3->setUseMaterial(true);
    p_CommonData->p_dynamicBox4->setMaterial(mat2);
    p_CommonData->p_dynamicBox4->setUseMaterial(true);
    // add mesh to ODE object
    p_CommonData->ODEBody1->setImageModel(p_CommonData->p_dynamicBox1);
    p_CommonData->ODEBody2->setImageModel(p_CommonData->p_dynamicBox2);
    p_CommonData->ODEBody3->setImageModel(p_CommonData->p_dynamicBox3);
    p_CommonData->ODEBody4->setImageModel(p_CommonData->p_dynamicBox4);
    // create a dynamic model of the ODE object
    p_CommonData->ODEBody1->createDynamicBox(boxSize1, boxSize1, boxSize1);
    p_CommonData->ODEBody2->createDynamicBox(boxSize2, boxSize2, 2*boxSize2);
    p_CommonData->ODEBody3->createDynamicBox(boxSize3, boxSize3, 3*boxSize3);
    p_CommonData->ODEBody4->createDynamicBox(boxSize3, boxSize3, 2*boxSize3);

    chai3d::cMatrix3d zeroRot; zeroRot.set(0,0,0,0,0,0,0,0,0);
    p_CommonData->ODEBody1->setLocalRot(zeroRot);
    p_CommonData->ODEBody2->setLocalRot(zeroRot);
    p_CommonData->ODEBody3->setLocalRot(zeroRot);
    p_CommonData->ODEBody4->setLocalRot(zeroRot);

    //put things where they go for start of experiment
    // Put the other blocks out of view
    p_CommonData->ODEBody1->setLocalPos(p_CommonData->box1InitPos);
    p_CommonData->ODEBody2->setLocalPos(p_CommonData->box2InitPos);
    p_CommonData->ODEBody3->setLocalPos(p_CommonData->box3InitPos);

    // Put standard block on the table
    p_CommonData->ODEBody4->setLocalPos(0.025,0,-0.05);
    p_CommonData->ODEBody4->setLocalRot(zeroRot);
    p_CommonData->sizeWeightStandardMass = mass2;
    p_CommonData->ODEBody4->setMass(mass2);
}

// was for general mass demo and subjective CHI exp
void haptics_thread::SetDynEnvironDemo()
{
    // create the visual boxes on the dynamicbox meshes
    cCreateBox(p_CommonData->p_dynamicBox1, boxSize1, boxSize1, boxSize1); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox2, boxSize2, boxSize2, boxSize2); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicBox3, boxSize3, boxSize3, boxSize3); // make mesh a box

    // create the visual for the position scaled dynamic boxes
    cCreateBox(p_CommonData->p_dynamicScaledBox1, boxSize1, boxSize1, boxSize1); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicScaledBox2, boxSize2, boxSize2, boxSize2); // make mesh a box
    cCreateBox(p_CommonData->p_dynamicScaledBox3, boxSize3, boxSize3, boxSize3); // make mesh a box

    // setup collision detectorsfor the dynamic objects
    p_CommonData->p_dynamicBox1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox2->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_dynamicBox3->createAABBCollisionDetector(toolRadius);

    // define material properties for box 1
    chai3d::cMaterial mat1;
    mat1.setRedCrimson();
    mat1.setStiffness(stiffness1);
    mat1.setLateralStiffness(latStiffness1);
    mat1.setDynamicFriction(dynFriction1);
    mat1.setStaticFriction(friction1);
    mat1.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox1->setMaterial(mat1);
    p_CommonData->p_dynamicBox1->setUseMaterial(true);

    // define material properties for box 2
    chai3d::cMaterial mat2;
    mat2.setBlueRoyal();
    mat2.setStiffness(stiffness2);
    mat2.setLateralStiffness(latStiffness2);
    mat2.setDynamicFriction(dynFriction2);
    mat2.setStaticFriction(friction2);
    mat2.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox2->setMaterial(mat2);
    p_CommonData->p_dynamicBox2->setUseMaterial(true);

    // define material properties for box 3
    chai3d::cMaterial mat3;
    mat3.setGreenLawn();
    mat3.setStiffness(stiffness3);
    mat3.setLateralStiffness(latStiffness3);
    mat3.setDynamicFriction(dynFriction3);
    mat3.setStaticFriction(friction3);
    mat3.setUseHapticFriction(true);
    p_CommonData->p_dynamicBox3->setMaterial(mat3);
    p_CommonData->p_dynamicBox3->setUseMaterial(true);

    // add mesh to ODE object
    p_CommonData->ODEBody1->setImageModel(p_CommonData->p_dynamicBox1);
    p_CommonData->ODEBody2->setImageModel(p_CommonData->p_dynamicBox2);
    p_CommonData->ODEBody3->setImageModel(p_CommonData->p_dynamicBox3);

    // create a dynamic model of the ODE object
    p_CommonData->ODEBody1->createDynamicBox(boxSize1, boxSize1, boxSize1);
    p_CommonData->ODEBody2->createDynamicBox(boxSize2, boxSize2, boxSize2);
    p_CommonData->ODEBody3->createDynamicBox(boxSize3, boxSize3, boxSize3);

    // set mass of box
    p_CommonData->ODEBody1->setMass(p_CommonData->sliderWeight);
    p_CommonData->ODEBody2->setMass(mass2);
    p_CommonData->ODEBody3->setMass(mass3);

    // set position of box
    p_CommonData->ODEBody1->setLocalPos(p_CommonData->box1InitPos);
    p_CommonData->ODEBody2->setLocalPos(p_CommonData->box2InitPos); //out of view
    p_CommonData->ODEBody3->setLocalPos(p_CommonData->box3InitPos);
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
    p_CommonData->p_world->addChild(p_CommonData->p_expFrictionBox);
    p_CommonData->p_world->addChild(m_tool0);
    p_CommonData->p_world->addChild(m_tool1);
    p_CommonData->p_world->addChild(finger);
}

void haptics_thread::RenderTwoFriction()
{    
    cCreateBox(p_CommonData->p_frictionBox1, .08, .08, .01); // make mesh a box
    cCreateBox(p_CommonData->p_frictionBox2, .08, .08, .01); // make mesh a box
    p_CommonData->p_frictionBox1->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_frictionBox2->createAABBCollisionDetector(toolRadius);
    p_CommonData->p_frictionBox1->setLocalPos(0,.08, 0);
    p_CommonData->p_frictionBox2->setLocalPos(0,-.08, 0);

    p_CommonData->p_frictionBox1->m_material->setStiffness(300);
    p_CommonData->p_frictionBox1->m_material->setLateralStiffness(600);
    p_CommonData->p_frictionBox1->m_material->setStaticFriction(0.25);
    p_CommonData->p_frictionBox1->m_material->setDynamicFriction(0.25*0.9);

    p_CommonData->p_frictionBox2->m_material->setStiffness(300);
    p_CommonData->p_frictionBox2->m_material->setLateralStiffness(600);
    p_CommonData->p_frictionBox2->m_material->setStaticFriction(0.8);
    p_CommonData->p_frictionBox2->m_material->setDynamicFriction(0.8*.9);

    // try creating texture box
    p_CommonData->p_textureBox->loadFromFile("./Resources/Texture.obj");
    p_CommonData->p_textureBox->setLocalPos(0,0,0);
    p_CommonData->p_textureBox->rotateAboutLocalAxisDeg(-1,0,0,90);
    p_CommonData->p_textureBox->rotateAboutLocalAxisDeg(0,1,0, 90);

    p_CommonData->p_textureBox->setUseVertexColors(true);
    chai3d::cColorf textureBoxColor;
    textureBoxColor.setBlue();
    p_CommonData->p_textureBox->setVertexColor(textureBoxColor);
    p_CommonData->p_textureBox->m_material->m_ambient.set(0.1, 0.1, 0.1);
    p_CommonData->p_textureBox->m_material->m_diffuse.set(0.3, 0.3, 0.3);
    p_CommonData->p_textureBox->m_material->m_specular.set(1.0, 1.0, 1.0);
    p_CommonData->p_textureBox->setUseMaterial(true);

    // compute a boundary box
    p_CommonData->p_textureBox->computeBoundaryBox(true);

    // compute collision detection algorithm
    p_CommonData->p_textureBox->createAABBCollisionDetector(toolRadius);

    // define a default stiffness for the object
    p_CommonData->p_textureBox->setTransparencyLevel(1, true, true);

    p_CommonData->p_textureBox->setStiffness(300, true);
    p_CommonData->p_textureBox->setFriction(.6, .5, TRUE);

    p_CommonData->p_world->addChild(p_CommonData->p_frictionBox1);
    p_CommonData->p_world->addChild(p_CommonData->p_frictionBox2);
    p_CommonData->p_world->addChild(p_CommonData->p_textureBox);
    p_CommonData->p_world->addChild(m_tool0);
    p_CommonData->p_world->addChild(m_tool1);
    p_CommonData->p_world->addChild(finger);
    p_CommonData->p_world->addChild(scaledFinger);
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

void haptics_thread::WriteDataToFile()
{
    p_CommonData->recordFlag = false;
    QString tempFreq;
    tempFreq.setNum(p_CommonData->bandSinFreq);
    //write data to file when we are done
    std::ofstream file;
    file.open(p_CommonData->dir.toStdString() + "/" + p_CommonData->fileName.toStdString() + tempFreq.toStdString() + ".txt");
    for (int i=0; i < p_CommonData->dataRecorderVector.size(); i++)
    {
        //[0] is distal finger, [1] is toward middle finger, [2] is away from finger pad
        file << p_CommonData->dataRecorderVector[i].time << "," << " "

        << p_CommonData->dataRecorderVector[i].pos0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].pos0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].pos0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut0[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut0[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut0[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos0.x() << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos0.y() << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos0.z() << "," << " "

        << p_CommonData->dataRecorderVector[i].pos1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].pos1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].pos1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].desiredPos1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForce1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].VRInteractionForceGlobal1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorAngles1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].jointAngles1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].motorTorque1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut1[0] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut1[1] << "," << " "
        << p_CommonData->dataRecorderVector[i].voltageOut1[2] << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos1.x() << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos1.y() << "," << " "
        << p_CommonData->dataRecorderVector[i].magTrackerPos1.z() << "," << " "

        << p_CommonData->dataRecorderVector[i].deviceRotation0(0,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(0,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(0,2) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(1,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(1,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(1,2) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(2,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(2,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation0(2,2) << "," << " "

        << p_CommonData->dataRecorderVector[i].deviceRotation1(0,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(0,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(0,2) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(1,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(1,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(1,2) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(2,0) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(2,1) << "," << " "
        << p_CommonData->dataRecorderVector[i].deviceRotation1(2,2) << "," << " "


        << p_CommonData->dataRecorderVector[i].pairNo << "," << " "

        << std::endl;
    }
    file.close();
    p_CommonData->dataRecorderVector.clear();
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
    p_CommonData->p_world->addChild(m_tool0);
    p_CommonData->p_world->addChild(m_tool1);
    p_CommonData->p_world->addChild(finger);

    double tissueNomStiffness = 300;
    double staticFriction = 0.6;
    double dynamicFriction = staticFriction*0.9;
    double vertOffset = 0.03;
    //----------------------------------------------Create Tissue One---------------------------------------------------
    // add object to world
    p_CommonData->p_world->addChild(p_CommonData->p_tissueOne);

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

//    p_CommonData->p_tissueOne->m_material->setStiffness(tissueNomStiffness);
//    p_CommonData->p_tissueOne->m_material->setLateralStiffness(1.5*tissueNomStiffness);
//    p_CommonData->p_tissueOne->m_material->setStaticFriction(staticFriction);
//    p_CommonData->p_tissueOne->m_material->setDynamicFriction(dynamicFriction);
    //----------------------------------------------Create Tissue Two---------------------------------------------------
    // add object to world
    p_CommonData->p_world->addChild(p_CommonData->p_tissueTwo);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueThree);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueFour);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueFive);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueSix);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueSeven);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueEight);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueNine);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueTen);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueEleven);

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
    p_CommonData->p_world->addChild(p_CommonData->p_tissueTwelve);

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
    p_CommonData->p_world->addChild(p_CommonData->p_indicator);

    p_CommonData->p_indicator->setLocalPos(0,0,-.026+vertOffset);

    //load the object from file
    p_CommonData->p_indicator->loadFromFile("./Resources/tissue_indicator OBJ/tissue_indicator.obj");

    p_CommonData->p_indicator->setHapticEnabled(false);
    chai3d::cMaterial mat0;
    mat0.setRed();
    p_CommonData->p_indicator->setMaterial(mat0);
    p_CommonData->p_indicator->setTransparencyLevel(1);
}

void haptics_thread::AddImpulseDisp(chai3d::cVector3d &indexForce, chai3d::cVector3d &thumbForce)
{
    // check delay since last impulse
    double delay = p_CommonData->impulseDelayClock.getCurrentTimeSeconds();
    if (delay > 0.5)
    {
        p_CommonData->impulseDelayClock.reset();
        p_CommonData->impulseDelayClock.start();
        p_CommonData->impulseClock.reset();
        p_CommonData->impulseClock.start();
    }

    double t = p_CommonData->impulseClock.getCurrentTimeSeconds();
    if (t > 1.5)
        t = 0;
    double desPeak = 3;
    double c1 = 5.0;
    double c2 = 50.0;
    double tPeak = -log(c1/c2)/(c2-c1);
    double rawMag = exp(-c1*tPeak) - exp(-c2*tPeak);
    double A = desPeak/rawMag;
    double impulse = A*(exp(-c1*t) - exp(-c2*t));


    indexForce = impulse*p_CommonData->globalImpulseDir;
    thumbForce = impulse*p_CommonData->globalImpulseDir;
}

void haptics_thread::AddImpulseTorqueDisp(chai3d::cVector3d &indexForce, chai3d::cVector3d &thumbForce)
{
    // check delay since last impulse
    double delay = p_CommonData->impulseTorqueDelayClock.getCurrentTimeSeconds();
    if (delay > 0.5)
    {
        p_CommonData->impulseTorqueDelayClock.reset();
        p_CommonData->impulseTorqueDelayClock.start();
        p_CommonData->impulseTorqueClock.reset();
        p_CommonData->impulseTorqueClock.start();
    }

    // determine the pulse magnitude
    double t = p_CommonData->impulseTorqueClock.getCurrentTimeSeconds();
    if (t > 1.5)
        t = 0;
    double desPeak = 3;
    double c1 = 5.0;
    double c2 = 50.0;
    double tPeak = -log(c1/c2)/(c2-c1);
    double rawMag = exp(-c1*tPeak) - exp(-c2*tPeak);
    double A = desPeak/rawMag;
    double impulse = A*(exp(-c1*t) - exp(-c2*t));

    // determine the direction of the torque impulse in world coordinates
    chai3d::cVector3d vecIndexToCenter = position1 - position0; vecIndexToCenter.normalize();
    chai3d::cVector3d vecThumbToCenter = position0 - position1; vecThumbToCenter.normalize();

    chai3d::cVector3d crossIndex;
    chai3d::cVector3d crossThumb;

    // to determine torque displacements, take cross product with torque direction
    vecIndexToCenter.crossr(p_CommonData->globalImpulseDir, crossIndex);
    vecThumbToCenter.crossr(p_CommonData->globalImpulseDir, crossThumb);

    indexForce = impulse*crossIndex;
    thumbForce = impulse*crossThumb;
}



