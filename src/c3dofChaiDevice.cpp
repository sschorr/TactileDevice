/* This file reads in data for two magnetic tracker tips on the Ascension tracker
 * and makes the data available as a custom Chai3d device.  It does not provide
 * any force output. */



//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Your name, institution, or company name.
    \version   3.0.0 $Rev: 1244 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "c3dofChaiDevice.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
    INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

    Please review header file c3dofChaiDevice.h for some initial
    guidelines about how to implement your own haptic device using this
    template.

    When ready, simply completed the next 11 documented steps described here
    bellow.
*/
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------



//==============================================================================
/*!
    Constructor of c3dofChaiDevice.
*/
//==============================================================================
c3dofChaiDevice::c3dofChaiDevice(unsigned int a_deviceNumber)
    : ourMagTracker(a_deviceNumber)
{
    // the connection to your device has not yet been established.
    m_deviceReady = false;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 1:

        Here you should define the specifications of your device.
        These values only need to be estimates. Since haptic devices often perform
        differently depending of their configuration within their workspace,
        simply use average values.
    */
    ////////////////////////////////////////////////////////////////////////////

    trackerNo = a_deviceNumber;
    //--------------------------------------------------------------------------
    // NAME:
    //--------------------------------------------------------------------------

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "My Name";

    // name of your device
    m_specifications.m_modelName                     = "My Custom Device";


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = 5.0;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = 0.2;     // [N*m]


    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce               = 3.0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = 1000.0; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = 1.0;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 1000;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = 0.2;     // [m]

    // the maximum opening angle of the gripper
    m_specifications.m_gripperMaxAngleRad             = cDegToRad(30.0);


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices"
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////

    // Maximum recommended linear damping factor Kv
    m_specifications.m_maxLinearDamping			      = 20.0;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping			  = 0.0;	  // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping		  = 0.0; // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = true;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = true;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = true;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = true;

    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;

    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;


    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 2:

        Here, you shall  implement code which tells the application if your
        device is actually connected to your computer and can be accessed.
        In practice this may be consist in checking if your I/O board
        is active or if your drivers are available.

        If your device can be accessed, set:
        m_systemAvailable = true;

        Otherwise set:
        m_systemAvailable = false;

        Your actual code may look like:

        bool result = checkIfMyDeviceIsAvailable()
        m_systemAvailable = result;

        If want to support multiple devices, using the method argument
        a_deviceNumber to know which device to setup
    */
    ////////////////////////////////////////////////////////////////////////////


    // *** INSERT YOUR CODE HERE ***
    m_MyVariable = 0;

    scaleFactor = 1.0;

    m_deviceAvailable = false; // this value should become 'true' when the device is available.
    m_deviceAvailable = true; // my code making device available
}


//==============================================================================
/*!
    Destructor of c3dofChaiDevice.
*/
//==============================================================================
c3dofChaiDevice::~c3dofChaiDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
}


//==============================================================================
/*!
    Open connection to your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 3:

        Here you shall implement to open a connection to your
        device. This may include opening a connection to an interface board
        for instance or a USB port.

        If the connection succeeds, set the variable 'result' to true.
        otherwise, set the variable 'result' to false.

        Verify that your device is calibrated. If your device
        needs calibration then call method calibrate() for wich you will
        provide code in STEP 5 further bellow.
    */
    ////////////////////////////////////////////////////////////////////////////
    bool result = C_ERROR; // this value will need to become "C_SUCCESS" for the device to be marked as ready.

    // *** INSERT YOUR CODE HERE ***
    // result = openConnectionToMyDevice();

    result = C_SUCCESS; // my code saying connection success
    // update device status
    if (result)
    {
        m_deviceReady = true;
        return (C_SUCCESS);
    }
    else
    {
        m_deviceReady = false;
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    Close connection to your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 4:

        Here you shall implement code that closes the connection to your
        device.

        If the operation fails, simply set the variable 'result' to C_ERROR   .
        If the connection succeeds, set the variable 'result' to C_SUCCESS.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

    // *** INSERT YOUR CODE HERE ***
    // result = closeConnectionToMyDevice()

    // update status
    m_deviceReady = false;

    return (result);
}


//==============================================================================
/*!
    Calibrate your device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::calibrate(bool a_forceCalibration)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 5:

        Here you shall implement code that handles a calibration procedure of the
        device. In practice this may include initializing the registers of the
        encoder counters for instance.

        If the device is already calibrated and  a_forceCalibration == false,
        the method may immediately return without further action.
        If a_forceCalibration == true, then the calibrartion procedure
        shall be executed even if the device has already been calibrated.

        If the calibration procedure succeeds, the method returns C_SUCCESS,
        otherwise return C_ERROR.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()

    return (result);
}


//==============================================================================
/*!
    Returns the number of devices available from this class of device.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
unsigned int c3dofChaiDevice::getNumDevices()
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 6:

        Here you shall implement code that returns the number of available
        haptic devices of type "c3dofChaiDevice" which are currently connected
        to your computer.

        In practice you will often have either 0 or 1 device. In which case
        the code here below is already implemented for you.

        If you have support more than 1 devices connected at the same time,
        then simply modify the code accordingly so that "numberOfDevices" takes
        the correct value.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

    int numberOfDevices = 1;  // At least set to 1 if a device is available.

    // numberOfDevices = getNumberOfDevicesConnectedToTheComputer();

    return (numberOfDevices);
}


//==============================================================================
/*!
    Read the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::getPosition(cVector3d& a_position)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 7:

        Here you shall implement code that reads the position (X,Y,Z) from
        your haptic device. Read the values from your device and modify
        the local variable (x,y,z) accordingly.
        If the operation fails return an C_ERROR, C_SUCCESS otherwise

        Note:
        For consistency, units must be in meters.
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.
    */
    ////////////////////////////////////////////////////////////////////////////



    bool result = C_SUCCESS;
    double x1, y1, z1, x2, y2, z2, xc, yc, zc, x, y, z;
    static double counter = 0;
    counter = counter ++;
    x = 0.06; y = .09*sin(.0001*counter);
    z = -0.15; // + .05*sin(.0001*counter);

    if (trackerNo == 1)
    {
        y = y + 0.02;
    }

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***
    // these axes align assuming the box is facing you and the chord of the tracker faces the box.
#ifdef MAGTRACKER
    // get position of this tracker
    pose = poseCache;
    pos = pose.getLocalPos();

    // get position of the other tracker
    otherPose = otherPoseCache;
    otherPos = otherPose.getLocalPos();

    x1 = pos.x(); y1 = pos.y(); z1 = pos.z();
    x2 = otherPos.x(); y2 = otherPos.y(); z2 = otherPos.z();
#endif

    centerPoint.set((x1+x2)/2.0, (y1+y2)/2.0, (z1+z2)/2.0);

    // scale by scale factor
    xc = centerPoint.x()*scaleFactor; yc = centerPoint.y()*scaleFactor; zc = centerPoint.z()*scaleFactor;
    scaledCenterPoint.set(xc, yc, zc);

    x = xc+(x1-centerPoint.x());
    y = yc+(y1-centerPoint.y());
    z = zc+(z1-centerPoint.z());

    // store new position values
    a_position.set(x, y, z);

    // transformation to go from mag tracker at back, to at finger tip (x red, y green, z blue)
    chai3d::cVector3d posOffsetinB;
    if (trackerNo == 0)
    {
        posOffsetinB.set(-.060, 0, .018);
    }

    if (trackerNo == 1)
    {
        posOffsetinB.set(-.067, 0, .020); //Thumb dimensions
        //posOffsetinB.set(-.060, 0, .018); //index dimensions
    }

    chai3d::cMatrix3d R_BtoA = pose.getLocalRot();
    chai3d::cVector3d posOffsetinA;
    R_BtoA.mulr(posOffsetinB, posOffsetinA);
    a_position = a_position + posOffsetinA;

    // estimate linear velocity
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}


//==============================================================================
/*!
    Read the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::getRotation(cMatrix3d& a_rotation)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 8:

        Here you shall implement code which reads the orientation frame from
        your haptic device. The orientation frame is expressed by a 3x3
        rotation matrix. The 1st column of this matrix corresponds to the
        x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
        The length of each column vector should be of length 1 and vectors need
        to be orthogonal to each other.

        Note:
        If your device is located in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        If your device has a stylus, make sure that you set the reference frame
        so that the x-axis corresponds to the axis of the stylus.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    cMatrix3d frame;
    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***
    // if the device does not provide any rotation capabilities
    // set the rotation matrix equal to the identity matrix.
    frame.identity();

#ifdef MAGTRACKER
    // try setting rotation from mag tracker
    frame = poseCache.getLocalRot();
#endif

    if (trackerNo == 0)
    {
        frame.rotateAboutLocalAxisDeg(0,1,0,9); //edit hereif change angle of finger
    }
    if (trackerNo == 1)
    {
        frame.rotateAboutLocalAxisDeg(0,1,0,9);
    }


    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // exit
    return (result);
}


//==============================================================================
/*!
    Read the gripper angle in radian.

    \param   a_angle  Return value.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::getGripperAngleRad(double& a_angle)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***

    // return gripper angle in radian
    a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 10:

        Here you may implement code which sends a force (fx,fy,fz),
        torque (tx, ty, tz) and/or gripper force (gf) command to your haptic device.

        If your device does not support one of more of the force, torque and
        gripper force capabilities, you can simply ignore them.

        Note:
        For consistency, units must be in Newtons and Newton-meters
        If your device is placed in front of you, the x-axis is pointing
        towards you (the operator). The y-axis points towards your right
        hand side and the z-axis points up towards the sky.

        For instance: if the force = (1,0,0), the device should move towards
        the operator, if the force = (0,0,1), the device should move upwards.
        A torque (1,0,0) would rotate the handle counter clock-wise around the
        x-axis.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    double tx = a_torque(0);
    double ty = a_torque(1);
    double tz = a_torque(2);

    double gf = a_gripperForce;

    // *** INSERT YOUR CODE HERE ***

    // setForceToMyDevice(fx, fy, fz);
    // setTorqueToMyDevice(tx, ty, tz);
    // setForceToGripper(fg);


    // exit
    return (result);
}


//==============================================================================
/*!
    Read the status of the user switch [__true__ = \e ON / __false__ = \e OFF].

    \param   a_switchIndex  index number of the switch.
    \param   a_status result value from reading the selected input switch.

    \return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c3dofChaiDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status of one or
        more user switches on your device. An application may request to read the status
        of a switch by passing its index number. The primary user switch mounted
        on the stylus of a haptic device will receive the index number 0. The
        second user switch is referred to as 1, and so on.

        The return value of a switch (a_status) shall be equal to "true" if the button
        is pressed or "false" otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    a_status = false;  // a_status = getUserSwitchOfMyDevice(a_switchIndex)

    return (result);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
