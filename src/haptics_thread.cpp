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
    // X is toward camera, pos y is to right, pos z is up
    p_CommonData->p_camera->set( chai3d::cVector3d (0.35/2, 0, -.03),    // camera position (eye)
                                 chai3d::cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                                 chai3d::cVector3d (0.0, 0.0, -1.0));   // direction of the "up" vector

    // create a light source and attach it to the camera
    light = new chai3d::cDirectionalLight(world);
    world->addChild(light);   // insert light source inside world
    light->setEnabled(true);                   // enable light source
    light->setDir(chai3d::cVector3d(-2.0, -0.5, 1.0));  // define the direction of the light beam

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

    /* uncomment this if we want to use 2 tools
    m_tool1 = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool1); //insert the tool into the world
    m_tool1->setRadius(toolRadius);
    m_tool1->setHapticDevice(p_CommonData->chaiMagDevice1); // connect the haptic device to the tool
    m_tool1->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool1->start(); */

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

    // create a box and give it physical properties
    meshBox = new chai3d::cMesh();  // create a mesh for a box
    cCreateBox(meshBox, .07, .07, .035); // make mesh a box
    meshBox->createAABBCollisionDetector(toolRadius); // create collision detector
    world->addChild(meshBox); // add to world
    meshBox->setLocalPos(0,0,.02); // set the position
    // give the box physical properties
    meshBox->m_material->setStiffness(200);
    meshBox->m_material->setStaticFriction(0.5);
    meshBox->m_material->setDynamicFriction(0.5);
    meshBox->m_material->setUseHapticFriction(true);

    // create a finger object
    finger = new chai3d::cMultiMesh(); // create a virtual mesh
    world->addChild(finger); // add object to world
    finger->setShowFrame(false);
    finger->setFrameSize(0.05);
    finger->setLocalPos(0,0,0);
    // load an object file
    if(cLoadFileOBJ(finger, "FingerModel.obj")){
        qDebug() << "finger file loaded";
    }
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

    // GENERAL HAPTICS INITS=================================
    // Ensure the device is not controlling to start
    p_CommonData->sliderControlMode = false;
    p_CommonData->VRControlMode = false;
    p_CommonData->wearableDelta->SetDesiredForce(Eigen::Vector3d(0,0,0));
    neutralPos = Eigen::Vector3d(0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180));
    p_CommonData->wearableDelta->SetDesiredPos(neutralPos); // kinematic neutral position
    firstTimeInSin = true;

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

    // init first contact variables
    decaySinAmpMax = 2;
    decaySinFreq = 150;
    decaySinExp = -20;
    decaySinScale = 2;
    firstTouch = true;
    decaySinAmp = 0;

    //init bandwidth variables
    bandSinAmp = 3;
    bandSinFreq = 3;
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

            if (p_CommonData->VRControlMode == true)
            {
                ComputeVRDesiredDevicePos();
            }
            else if (p_CommonData->sinControlMode == true)
            {
                Eigen::Vector3d inputAxis(1,0,0);
                CommandSinPos(inputAxis);
            }

            p_CommonData->wearableDelta->PositionController();

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
                //RecordData();
            }

            // restart rateClock
            rateClock.reset();
            rateClock.start();
        }        
    }
    // If we are terminating, delete the haptic device to set outputs to 0
    delete p_CommonData->wearableDelta;
}

void haptics_thread::CommandSinPos(Eigen::Vector3d inputMotionAxis)
{
    //perform check on validity of axis
    double inputAxisMag = inputMotionAxis.norm();
    if ((inputAxisMag < 1.01) & (inputAxisMag > 0.99))
    {
        if(firstTimeInSin)
        {
            startTime = overallClock.getCurrentTimeSeconds();
            firstTimeInSin = false;
        }

        double currTime = overallClock.getCurrentTimeSeconds() - startTime;
        Eigen::Vector3d sinPos = (bandSinAmp*sin(2*PI*bandSinFreq*currTime))*inputMotionAxis + neutralPos;
        p_CommonData->wearableDelta->SetDesiredPos(sinPos);
    }
    else
    {
        qDebug("invalid motion axis");
    }
}

void haptics_thread::ComputeVRDesiredDevicePos()
{
    //update Chai3D parameters
    // compute global reference frames for each object
    world->computeGlobalPositions(true);

    // update position and orientation of tool (and sphere that represents tool)
    m_tool0->updatePose();
    chai3d::cVector3d position0; chai3d::cMatrix3d rotation0;
    chai3d::cMatrix3d fingerRotation0; chai3d::cMatrix3d deviceRotation0;
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

    /* use this if two tools (haptic proxies) are desired
    m_tool1->updatePose();
    chai3d::cVector3d position1; chai3d::cMatrix3d rotation1;
    chai3d::cMatrix3d fingerRotation1; chai3d::cMatrix3d deviceRotation1;
    p_CommonData->chaiMagDevice1->getPosition(position1);
    p_CommonData->chaiMagDevice1->getRotation(rotation1);
    //m_curSphere1->setLocalPos(position1);
    //m_curSphere1->setLocalRot(rotation1);
    m_tool1->computeInteractionForces();*/

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

