#include "c3DOFdevice.h"


c3DOFDevice::c3DOFDevice()
{
    this->desiredForce << 0,0,0;
    this->neutralPos = Eigen::Vector3d(0,0,L_LA*sin(45*PI/180)+L_UA*sin(45*PI/180));
    this->motorTorques << 0,0,0;
    this->jointTorques << 0,0,0;
}

c3DOFDevice::~c3DOFDevice()
{
    delete this->motor_1;
    delete this->motor_2;
    delete this->motor_3;
}

int c3DOFDevice::Init3DOFDeviceEnc()
{
    motor_1 = new cMotorController(1);
    motor_1->InitEncoder();
    motor_2 = new cMotorController(2);
    motor_2->InitEncoder();
    motor_3 = new cMotorController(3);
    motor_3->InitEncoder();

    return 0;
}

Eigen::Vector3d c3DOFDevice::GetMotorAngles()
{
    Eigen::Vector3d returnAngles(3);
    returnAngles << motor_1->GetMotorAngle(),
                    motor_2->GetMotorAngle(),
                    motor_3->GetMotorAngle();
    return returnAngles;
}

void c3DOFDevice::ZeroEncoders()
{
    motor_1->SetOffsetAngle();
    motor_2->SetOffsetAngle();
    motor_3->SetOffsetAngle();
}


Eigen::Vector3d c3DOFDevice::GetJointAngles()
{
    // assumes we are post calibration (encoders zerod)
    Eigen::Vector3d motorAngles = GetMotorAngles();
    Eigen::Vector3d jointAngles(3);

    // sets the initial condition tether length based on device dimensions and calibration angle
    double initialTethL = sqrt(pow((ATTACHL*sin(CALIBANGLE) - VERTOFFSET),2) + pow((HORIZOFFSET - ATTACHL*cos(CALIBANGLE)), 2));

    //increment through each motorController
    for (int i = 0; i <= 2; i = i+1)
    {
        double tethChange = MOTRAD*(-motorAngles[i]);

        // account for fact that third motor turns in the opposite direction
        if(i==2)
        {
            tethChange = MOTRAD*(motorAngles[i]);
        }
        double tethL = initialTethL + tethChange;

        double a = (tethL*tethL - VERTOFFSET*VERTOFFSET - HORIZOFFSET*HORIZOFFSET - ATTACHL*ATTACHL)/((-2)*ATTACHL*sqrt(HORIZOFFSET*HORIZOFFSET+VERTOFFSET*VERTOFFSET));
        double phi = atan2(VERTOFFSET, HORIZOFFSET);
        double phiMinusTheta = acos(a);
        double theta = -(-phiMinusTheta - phi);

        jointAngles[i] = theta;
    }

    return jointAngles;
}

Eigen::Vector3d c3DOFDevice::CalcInverseKinJoint()
{
    Eigen::Vector3d returnAngles;
    double x = this->desiredPos[0]; double y = this->desiredPos[1]; double z = this->desiredPos[2];
    double l = 9; double L = 9;
    double wb = 15;
    double ub = 30;
    double up = 15;
    double wp = 7.5;
    double sp = 25.93;

    double a = wb - up;
    double b = sp/2.0 - sqrt(3.0)/2.0*wb;
    double c = wp - 0.5*wb;

    double E1 = 2.0*L*(-x + a);
    double F1 = 2.0*(-z)*L;
    double G1 = pow(y,2.0) + pow(x,2.0) + pow(z,2.0) + pow(a,2.0) + pow(L,2.0) -2.0*x*a - pow(l,2.0);

    double E2 = -L*(sqrt(3.0)*(-y+b) - x + c);
    double F2 = -2.0*z*L;
    double G2 =pow(y,2.0) + pow(x,2.0) + pow(z,2.0) + pow(b,2.0) + pow(c,2.0) + pow(L,2.0) + 2.0*((-y)*b - x*c) - pow(l,2.0);

    double E3 = L*(sqrt(3.0)*(-y-b) + x - c);
    double F3 = 2.0*(-z)*L;
    double G3 = pow(y,2.0) + pow(x,2.0) + pow(z,2.0) + pow(c,2.0) + pow(L,2.0) + 2.0*(y*b-x*c) - pow(l,2.0);

    double t1 = (-F1 - sqrt(pow(E1,2.0) + pow(F1,2.0) - pow(G1,2.0)))/(G1 - E1);
    double theta1 = 2.0*atan(t1);

    double t2 = (-F2 - sqrt(pow(E2,2.0) + pow(F2,2.0) - pow(G2,2.0)))/(G2 - E2);
    double theta3 = 2.0*atan(t2);

    double t3 = (-F3 - sqrt(pow(E3,2.0) + pow(F3,2.0) - pow(G3,2.0)))/(G3 - E3);
    double theta2 = 2.0*atan(t3);

    returnAngles << theta1, theta2, theta3;
    return returnAngles;
}

Eigen::Vector3d c3DOFDevice::GetCartesianPos()
{
    Eigen::Vector3d pos(3);
    Eigen::Vector3d jointAngles = GetJointAngles();

    // base side length
    double base = L_BASE*1.7321;
    double ee = L_EE*1.7321;

    double sqrt3 = sqrt(3.0); double tan60 = sqrt3; double sin30 = 0.5; double tan30 = 1/sqrt3;

    double t = (base-ee)*tan30/2;
    double y1 = -(t+L_LA*cos(jointAngles[0])); double z1 = -L_LA*sin(jointAngles[0]);
    double y2 = (t+L_LA*cos(jointAngles[2]))*sin30; double x2 = y2*tan60; double z2 = -L_LA*sin(jointAngles[2]);
    double y3 = (t+L_LA*cos(jointAngles[1]))*sin30; double x3 = -y3*tan60; double z3 = -L_LA*sin(jointAngles[1]);

    double dnm = (y2-y1)*x3-(y3-y1)*x2;
    double w1 = y1*y1 + z1*z1; double w2 = x2*x2 + y2*y2 + z2*z2; double w3 = x3*x3 + y3*y3 + z3*z3;

    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2;
    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
    double b2 = ((w2-w1)*x3-(w3-w1)*x2)/2;

    double a = a1*a1 + a2*a2 + dnm*dnm;
    double b = 2*(a1*b1+a2*(b2-y1*dnm)-z1*dnm*dnm);
    double c = (b2-y1*dnm)*(b2-y1*dnm)+b1*b1+dnm*dnm*(z1*z1-L_UA*L_UA);

    double d = b*b-4*a*c;
    if (d < 0)
    {
       qDebug() << "Invalid position";
    }

    double zQ = 0.5*(b+sqrt(d))/a;
    double xQ = (a2*zQ+b2)/dnm;
    double yQ = (a1*zQ+b1)/dnm;
    pos << xQ, yQ, zQ;

    return pos;
}

Eigen::Vector3d c3DOFDevice::CalcDesiredJointTorques(Eigen::Vector3d desiredForceArg)
{
    Eigen::Vector3d jointAngles = GetJointAngles();
    Eigen::Vector3d eePos = GetCartesianPos();

    //////////////////////////////////////////////////////////////////////
    /// Jacobian Calcs
    //////////////////////////////////////////////////////////////////////
    ///
    double phi1 = 0; double phi2 = 2*PI/3; double phi3 = 4*PI/3;

    Eigen::Matrix3d Rz1;
    Rz1 << cos(phi1), -sin(phi1), 0,
            sin(phi1), cos(phi1), 0,
            0, 0, 1;

    Eigen::Matrix3d Rz2;
    Rz2 << cos(phi2), -sin(phi2), 0,
            sin(phi2), cos(phi2), 0,
            0, 0, 1;

    Eigen::Matrix3d Rz3;
    Rz3 << cos(phi3), -sin(phi3), 0,
            sin(phi3), cos(phi3), 0,
            0, 0, 1;

    Eigen::Matrix3d Rtheta_1;
    Rtheta_1 << cos(-jointAngles[0]), 0, sin(-jointAngles[0]),
            0, 1, 0,
            -sin(-jointAngles[0]), 0, cos(-jointAngles[0]);

    Eigen::Matrix3d Rtheta_2;
    Rtheta_2 << cos(-jointAngles[1]), 0, sin(-jointAngles[1]),
            0, 1, 0,
            -sin(-jointAngles[1]), 0, cos(-jointAngles[1]);

    Eigen::Matrix3d Rtheta_3;
    Rtheta_3 << cos(-jointAngles[2]), 0, sin(-jointAngles[2]),
            0, 1, 0,
            -sin(-jointAngles[2]), 0, cos(-jointAngles[2]);

    Eigen::Vector3d L_BASE_Vector(L_BASE, 0, 0);
    Eigen::Vector3d L_LA_Vector(L_LA, 0, 0);
    Eigen::Vector3d L_EE_Vector(L_EE, 0, 0);

    // base origin to joint base
    Eigen::Vector3d O_F1 = Rz1*L_BASE_Vector;
    Eigen::Vector3d O_F2 = Rz2*L_BASE_Vector;
    Eigen::Vector3d O_F3 = Rz3*L_BASE_Vector;

    // lower link
    Eigen::Vector3d F1_E1 = Rz1*Rtheta_1*L_LA_Vector;
    Eigen::Vector3d F2_E2 = Rz2*Rtheta_2*L_LA_Vector;
    Eigen::Vector3d F3_E3 = Rz3*Rtheta_3*L_LA_Vector;

    // origin to end of lower link
    Eigen::Vector3d O_E1 = O_F1 + F1_E1;
    Eigen::Vector3d O_E2 = O_F2 + F2_E2;
    Eigen::Vector3d O_E3 = O_F3 + F3_E3;

    // end effector to end of second link
    Eigen::Vector3d Q_P1 = Rz1*L_EE_Vector;
    Eigen::Vector3d Q_P2 = Rz2*L_EE_Vector;
    Eigen::Vector3d Q_P3 = Rz3*L_EE_Vector;

    // end effector position multipliable vector
    Eigen::Vector3d eePosVector(eePos[0], eePos[1], eePos[2]);

    Eigen::Vector3d s1 = eePosVector + Q_P1 - O_E1;
    Eigen::Vector3d s2 = eePosVector + Q_P2 - O_E2;
    Eigen::Vector3d s3 = eePosVector + Q_P3 - O_E3;

    Eigen::Vector3d vec1(-L_LA*sin(jointAngles[0]), 0, L_LA*cos(jointAngles[0]));
    Eigen::Vector3d vec2(-L_LA*sin(jointAngles[1]), 0, L_LA*cos(jointAngles[1]));
    Eigen::Vector3d vec3(-L_LA*sin(jointAngles[2]), 0, L_LA*cos(jointAngles[2]));

    Eigen::Vector3d b1 = Rz1*vec1;
    Eigen::Vector3d b2 = Rz2*vec2;
    Eigen::Vector3d b3 = Rz3*vec3;

    Eigen::Matrix3d Jx;
    Jx << s1(0,0), s1(1,0), s1(2,0),
          s2(0,0), s2(1,0), s2(2,0),
          s3(0,0), s3(1,0), s3(2,0);

    Eigen::Matrix3d Jtheta;
    Jtheta << (s1.transpose()*b1), 0, 0,
               0, (s2.transpose()*b2), 0,
               0, 0, (s3.transpose()*b3);

    Eigen::Matrix3d J = Jx.inverse()*Jtheta;

    // Torque needed about joints = J'*F (positive in direction pushing "out", same as for joint angle measurement)
    Eigen::Vector3d Torque_Needed_NoSpring = J.transpose()*desiredForceArg;

    // Adjust the torque needed by the bias spring force
    double springTorStiff = SPRING_TORQUE/180*113*180/PI; // stiffness in mNm/rad
    Eigen::Vector3d Torque_Needed_WithSpring;
    Torque_Needed_WithSpring << Torque_Needed_NoSpring[0]-springTorStiff*(PI-jointAngles[0]),
                                Torque_Needed_NoSpring[1]-springTorStiff*(PI-jointAngles[1]),
                                Torque_Needed_NoSpring[2]-springTorStiff*(PI-jointAngles[2]);

    return Torque_Needed_WithSpring;
}

Eigen::Vector3d c3DOFDevice::CalcDesiredMotorTorques(Eigen::Vector3d desiredForceOutput)
{
    Eigen::Vector3d jointTorquesNeeded = CalcDesiredJointTorques(desiredForceOutput);
    Eigen::Vector3d jointAngles = GetJointAngles();

    // Describe vector from joint to tether attachment point
    Eigen::Vector3d vec_joint_teth1(ATTACHL*cos(jointAngles[0]), ATTACHL*sin(jointAngles[0]), 0);
    Eigen::Vector3d vec_joint_teth2(ATTACHL*cos(jointAngles[1]), ATTACHL*sin(jointAngles[1]), 0);
    Eigen::Vector3d vec_joint_teth3(ATTACHL*cos(jointAngles[2]), ATTACHL*sin(jointAngles[2]), 0);

    // Describe vector from tether to motor pulley
    Eigen::Vector3d vec_joint_mot1(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_joint_mot2(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_joint_mot3(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_teth_mot1 = vec_joint_mot1 - vec_joint_teth1;
    Eigen::Vector3d vec_teth_mot2 = vec_joint_mot2 - vec_joint_teth2;
    Eigen::Vector3d vec_teth_mot3 = vec_joint_mot3 - vec_joint_teth3;

    // normalize that vector
    vec_teth_mot1.normalize();
    vec_teth_mot2.normalize();
    vec_teth_mot3.normalize();

    // find cross products in order to determine necessary motor torque (will be [0, 0, z] due to 2D cross product)
    double cross1 = vec_joint_teth1.cross(vec_teth_mot1)[2];
    double cross2 = vec_joint_teth2.cross(vec_teth_mot2)[2];
    double cross3 = vec_joint_teth3.cross(vec_teth_mot3)[2];

    // fill in torque needed by motors for desired force
    Eigen::Vector3d torque;
    torque[0] = jointTorquesNeeded[0]*MOTRAD/cross1;
    torque[1] = jointTorquesNeeded[1]*MOTRAD/cross2;

    // turning the 3rd motor in the opposite direction pulls the cable
    torque[2] = -(jointTorquesNeeded[2]*MOTRAD/cross3);

    return torque;
}

Eigen::Vector3d c3DOFDevice::CalcDesiredMotorTorquesJointControl(Eigen::Vector3d desiredJointTorqueOutput)
{
    Eigen::Vector3d jointTorquesNeeded = desiredJointTorqueOutput; // assumes we've already accounted for springs
    Eigen::Vector3d jointAngles = GetJointAngles();

    // Describe vector from joint to tether attachment point
    Eigen::Vector3d vec_joint_teth1(ATTACHL*cos(jointAngles[0]), ATTACHL*sin(jointAngles[0]), 0);
    Eigen::Vector3d vec_joint_teth2(ATTACHL*cos(jointAngles[1]), ATTACHL*sin(jointAngles[1]), 0);
    Eigen::Vector3d vec_joint_teth3(ATTACHL*cos(jointAngles[2]), ATTACHL*sin(jointAngles[2]), 0);

    // Describe vector from tether to motor pulley
    Eigen::Vector3d vec_joint_mot1(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_joint_mot2(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_joint_mot3(HORIZOFFSET, VERTOFFSET, 0);
    Eigen::Vector3d vec_teth_mot1 = vec_joint_mot1 - vec_joint_teth1;
    Eigen::Vector3d vec_teth_mot2 = vec_joint_mot2 - vec_joint_teth2;
    Eigen::Vector3d vec_teth_mot3 = vec_joint_mot3 - vec_joint_teth3;

    // normalize that vector
    vec_teth_mot1.normalize();
    vec_teth_mot2.normalize();
    vec_teth_mot3.normalize();

    // find cross products in order to determine necessary motor torque (will be [0, 0, z] due to 2D cross product)
    double cross1 = vec_joint_teth1.cross(vec_teth_mot1)[2];
    double cross2 = vec_joint_teth2.cross(vec_teth_mot2)[2];
    double cross3 = vec_joint_teth3.cross(vec_teth_mot3)[2];

    // fill in torque needed by motors for desired force
    Eigen::Vector3d torque;
    torque[0] = jointTorquesNeeded[0]*MOTRAD/cross1;
    torque[1] = jointTorquesNeeded[1]*MOTRAD/cross2;

    // turning the 3rd motor in the opposite direction pulls the cable
    torque[2] = -(jointTorquesNeeded[2]*MOTRAD/cross3);

    return torque;
}

// Set the desired force
void c3DOFDevice::SetDesiredForce(Eigen::Vector3d desiredForceArg)
{
    this->desiredForce = desiredForceArg;
}

// Set the desired pos
void c3DOFDevice::SetDesiredPos(Eigen::Vector3d desiredPosArg)
{
    // limit workspace motion (plus or minus)
    double xLimit = 4; double yLimit = 4; double zLimit = 4;
    if (desiredPosArg[0] > xLimit) desiredPosArg[0] = xLimit;
    if (desiredPosArg[0] < -xLimit) desiredPosArg[0] = -xLimit;
    if (desiredPosArg[1] > yLimit) desiredPosArg[1] = yLimit;
    if (desiredPosArg[1] < -yLimit) desiredPosArg[1] = -yLimit;
    if (desiredPosArg[2] > (neutralPos[2] + zLimit)) desiredPosArg[2] = neutralPos[2] + zLimit;
    if (desiredPosArg[2] < (neutralPos[2] - zLimit)) desiredPosArg[2] = neutralPos[2] - zLimit;

    this->desiredPos = desiredPosArg;
}

// read the desired force
Eigen::Vector3d c3DOFDevice::ReadDesiredForce()
{
    return this->desiredForce;
}

// read the desired pos
Eigen::Vector3d c3DOFDevice::ReadDesiredPos()
{
    return this->desiredPos;
}

void c3DOFDevice::SetCartesianTorqueOutput(Eigen::Vector3d desiredForceOutput)
{
    Eigen::Vector3d desiredMotorTorques = CalcDesiredMotorTorques(desiredForceOutput);

    this->motor_1->SetOutputTorque(desiredMotorTorques[0]);
    this->motor_2->SetOutputTorque(desiredMotorTorques[1]);
    this->motor_3->SetOutputTorque(desiredMotorTorques[2]);
}

void c3DOFDevice::SetJointTorqueOutput(Eigen::Vector3d desJointTorqueOutput)
{
    motorTorques = CalcDesiredMotorTorquesJointControl(desJointTorqueOutput);

    this->motor_1->SetOutputTorque(motorTorques[0]);
    this->motor_2->SetOutputTorque(motorTorques[1]);
    this->motor_3->SetOutputTorque(motorTorques[2]);
}

Eigen::Vector3d c3DOFDevice::ReadVoltageOutput()
{
    Eigen::Vector3d returnVoltage(this->motor_1->voltageOutput, this->motor_2->voltageOutput, this->motor_3->voltageOutput);
    return returnVoltage;
}

void c3DOFDevice::PositionController(double Kp, double Kd)
{    
    double K_p = Kp;
    double K_d = Kd;

    static bool firstTimeThrough = true;
    static Eigen::Vector3d lastPos;
    static Eigen::Vector3d lastVel;
    Eigen::Vector3d desiredVel(0,0,0);
    double alpha = 0.5;

    Eigen::Vector3d currentPos = GetCartesianPos();
    Eigen::Vector3d desiredPos = ReadDesiredPos();
    if(firstTimeThrough)
    {
        lastPos = currentPos;
        lastVel << 0,0,0;
        firstTimeThrough = false;
    }

    Eigen::Vector3d currentVel = currentPos-lastPos;
    Eigen::Vector3d filteredVel = alpha*currentVel + (1-alpha)*lastVel;
    Eigen::Vector3d controllerForce = K_p*(desiredPos-currentPos) + K_d*(desiredVel-filteredVel);

    SetDesiredForce(controllerForce);
    SetCartesianTorqueOutput(ReadDesiredForce());

    lastVel = filteredVel;
    lastPos = currentPos;
}

void c3DOFDevice::JointController(double Kp, double Kd)
{
    static bool firstTimeThrough = true;
    static Eigen::Vector3d lastAngles;
    static Eigen::Vector3d lastAngVel;
    Eigen::Vector3d desAngleVel(0,0,0);
    double alpha = 0.5;

    Eigen::Vector3d jointAngles = GetJointAngles();
    Eigen::Vector3d desJointAngles = CalcInverseKinJoint();

    if(firstTimeThrough)
    {
        lastAngles = jointAngles;
        lastAngVel << 0,0,0;
        firstTimeThrough = false;
    }

    Eigen::Vector3d currAngVel = jointAngles-lastAngles;
    Eigen::Vector3d filteredVel = alpha*currAngVel + (1-alpha)*lastAngVel;
    jointTorques = Kp*(desJointAngles - jointAngles) + Kd*(desAngleVel-filteredVel);

    // Adjust the torque needed by the bias spring force
    double springTorStiff = SPRING_TORQUE/180*113*180/PI; // stiffness in mNm/rad
    jointTorques << jointTorques[0]-springTorStiff*(PI-jointAngles[0]),
                    jointTorques[1]-springTorStiff*(PI-jointAngles[1]),
                    jointTorques[2]-springTorStiff*(PI-jointAngles[2]);

    SetJointTorqueOutput(jointTorques);

    lastAngVel = filteredVel;
    lastAngles = jointAngles;
}

void c3DOFDevice::TurnOffControl()
{
    // create a zero force
    Eigen::Vector3d nullForce(0,0,0);
    // set the zero force as the desired force
    SetDesiredForce(nullForce);
    // set output torque based on zero force
    SetCartesianTorqueOutput(ReadDesiredForce());
}

void c3DOFDevice::ForceController()
{
    SetCartesianTorqueOutput(ReadDesiredForce());
}

void c3DOFDevice::TestMotorTorqueController()
{
    static double hapticTime = 0;

    // we gain about .001 second ever time through loop
    hapticTime = hapticTime + 0.001;

    double amp = 25;

    double torque1 = amp*sin(2*PI*.1*hapticTime);
    double torque2 = amp*sin(2*PI*.1*hapticTime);
    double torque3 = amp*sin(2*PI*.1*hapticTime);

    motor_1->SetOutputTorque(torque1);
    motor_2->SetOutputTorque(torque2);
    motor_3->SetOutputTorque(torque3);
}




