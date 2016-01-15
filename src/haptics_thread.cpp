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
    m_tool = new chai3d::cToolCursor(world); // create a 3D tool
    world->addChild(m_tool); //insert the tool into the world    
    toolRadius = 0.003; // set tool radius
    m_tool->setRadius(toolRadius);
    m_tool->setHapticDevice(p_CommonData->chaiMagDevice0); // connect the haptic device to the tool
    m_tool->setShowContactPoints(true, true, chai3d::cColorf(0,0,0)); // show proxy and device position of finger-proxy algorithm
    m_tool->start();

    //create a sphere to represent the tool
    m_curSphere = new chai3d::cShapeSphere(toolRadius);
    world->addChild(m_curSphere);
    m_curSphere->m_material->setGrayDarkSlate();
    m_curSphere->setShowFrame(true);
    m_curSphere->setFrameSize(0.05);


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

    // init first contact variables
    decaySinAmpMax = 2;
    decaySinFreq = 150;
    decaySinExp = -20;
    decaySinScale = 2;
    firstTouch = true;
    decaySinAmp = 0;
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

            //update Chai3D parameters
            // compute global reference frames for each object
            world->computeGlobalPositions(true);

            // update position and orientation of tool (and sphere that represents tool)
            m_tool->updatePose();
            chai3d::cVector3d position; chai3d::cMatrix3d rotation;
            chai3d::cMatrix3d fingerRotation; chai3d::cMatrix3d deviceRotation;
            p_CommonData->chaiMagDevice0->getPosition(position);
            p_CommonData->chaiMagDevice0->getRotation(rotation);
            m_curSphere->setLocalPos(position);
            m_curSphere->setLocalRot(rotation);

            // update position of finger to stay on proxy point
            // finger axis are not at fingerpad, so we want a translation along fingertip z axis
            chai3d::cVector3d fingerOffset(0,-0.006,0);
            fingerRotation = rotation;
            fingerRotation.rotateAboutLocalAxisDeg(0,0,1,90);            
            fingerRotation.rotateAboutLocalAxisDeg(1,0,0,90);
            finger->setLocalRot(fingerRotation);
            finger->setLocalPos(m_tool->m_hapticPoint->getGlobalPosProxy() + fingerRotation*fingerOffset);

            //computes the interaction force for the tool proxy point
            m_tool->computeInteractionForces();

            //perform transformation to get "device forces"
            lastComputedForce = m_tool->m_lastComputedGlobalForce;
            rotation.trans();
            deviceRotation.identity();
            deviceRotation.rotateAboutLocalAxisDeg(0,0,1,180);
            deviceRotation.trans();
            magTrackerLastComputedForce = rotation*lastComputedForce;
            deviceLastLastComputedForce = deviceLastComputedForce;
            deviceLastComputedForce = deviceRotation*rotation*lastComputedForce;

            //convert device "force" to a mapped position
            double forceToPosMult = 1;
            chai3d::cVector3d desiredPosMovement = forceToPosMult*deviceLastComputedForce;
            Eigen::Vector3d neutralPos = p_CommonData->wearableDelta->neutralPos;
            Eigen::Vector3d desiredPos(3);
            desiredPos << desiredPosMovement.x()+neutralPos[0], desiredPosMovement.y()+neutralPos[1], desiredPosMovement.z()+neutralPos[2];

            //create a contact vibration that depends on position or velocity
            // if we are registering a contact
            if (abs(deviceLastComputedForce.z()) > 0.00001)
            {
                // if this is the first time in contact
                if (firstTouch)
                {
                    // start the time ticking
                    decaySinTime = 0;
                    decaySinTime += 0.001;

                    // get the sinusoid amplitude based last velocity
                    p_CommonData->chaiMagDevice0->getLinearVelocity(estimatedVel);
                    this->decaySinAmp = this->decaySinScale*estimatedVel.z();

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

            desiredPos[2] = desiredPos[2]+computedPosAdd;
            // Perform position controller based on desired position
            p_CommonData->wearableDelta->SetDesiredPos(desiredPos);
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
    delete p_CommonData->wearableDelta;
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

