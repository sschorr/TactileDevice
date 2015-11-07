#include "c3dofdevice.h"


c3DOFDevice::c3DOFDevice()
{

}

c3DOFDevice::~c3DOFDevice()
{

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

QVector<double> c3DOFDevice::GetMotorAngles()
{
    QVector<double> returnAngles(3);
    returnAngles[0] = motor_1->GetMotorAngle();
    returnAngles[1] = motor_2->GetMotorAngle();
    returnAngles[2] = motor_3->GetMotorAngle();
    return returnAngles;
}

void c3DOFDevice::ZeroEncoders()
{
    motor_1->SetOffsetAngle();
    motor_2->SetOffsetAngle();
    motor_3->SetOffsetAngle();
}


QVector<double> c3DOFDevice::GetJointAngles()
{
    // assumes we are post calibration (encoders zerod)
    QVector<double> motorAngles = GetMotorAngles();
    QVector<double> jointAngles(3);

    // sets the initial condition tether length based on device dimensions and calibration angle
    double initialTethL = sqrt(pow((ATTACHL*sin(CALIBANGLE) - VERTOFFSET),2) + pow((HORIZOFFSET - ATTACHL*cos(CALIBANGLE)), 2));

    //increment through each motorController
    for (int i = 0; i <= 2; i = i+1)
    {
        double tethChange = MOTRAD*motorAngles[i];
        double tethL = initialTethL + tethChange;

        double a = (tethL*tethL - VERTOFFSET*VERTOFFSET - HORIZOFFSET*HORIZOFFSET - ATTACHL*ATTACHL)/((-2)*ATTACHL*sqrt(HORIZOFFSET*HORIZOFFSET+VERTOFFSET*VERTOFFSET));
        double phi = atan2(VERTOFFSET, HORIZOFFSET);
        double phiMinusTheta = acos(a);
        double theta = -(-phiMinusTheta - phi);
        jointAngles[i] = theta;
    }

    return jointAngles;
}

QVector<double> c3DOFDevice::GetCartesianPos()
{
    QVector<double> pos(3);

    // base side length
    double base = L_BASE*1.7321;
    double ee = L_EE*1.7321;

    double sqrt3 = sqrt(3); double tan60 = sqrt3; double sin30 = 0.5; double tan30 = 1/sqrt3;

    double t = (base-ee)*tan30/2;
    double y1 = -(t+L_LA*cos(GetJointAngles()[0])); double z1 = -L_LA*sin(GetJointAngles()[0]);
    double y2 = (t+L_LA*cos(GetJointAngles()[2]))*sin30; double x2 = y2*tan60; double z2 = -L_LA*sin(GetJointAngles()[2]);
    double y3 = (t+L_LA*cos(GetJointAngles()[1]))*sin30; double x3 = -y3*tan60; double z3 = -L_LA*sin(GetJointAngles()[1]);

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
    pos[0] = xQ; pos[1] = yQ; pos[2] = zQ;

    return pos;
}

QVector<double> c3DOFDevice::GetDesiredTorques(QVector<double> DesiredForce)
{
    QVector<double> torque;
    QVector<double> motorAngles = GetJointAngles();
    QVector<double> eePos = GetCartesianPos();
    double phi1 = 0; double phi2 = 2*PI/3; double phi3 = 4*PI/3;

    double Rz1Array[] = {
        cos(phi1), -sin(phi1), 0,
        sin(phi1), cos(phi1), 0,
        0, 0, 1};

    double Rz2Array[] = {
        cos(phi2), -sin(phi2), 0,
        sin(phi2), cos(phi2), 0,
        0, 0, 1};

    double Rz3Array[] = {
        cos(phi3), -sin(phi3), 0,
        sin(phi3), cos(phi3), 0,
        0, 0, 1};

    double Rtheta_1Array[] = {
        cos(-motorAngles[0]), 0, sin(-motorAngles[0]),
        0, 1, 0,
        -sin(-motorAngles[0]), 0, cos(-motorAngles[0])
    };

    double Rtheta_2Array[] = {
        cos(-motorAngles[1]), 0, sin(-motorAngles[1]),
        0, 1, 0,
        -sin(-motorAngles[1]), 0, cos(-motorAngles[1])
    };

    double Rtheta_3Array[] = {
        cos(-motorAngles[2]), 0, sin(-motorAngles[2]),
        0, 1, 0,
        -sin(-motorAngles[2]), 0, cos(-motorAngles[2])
    };


    QGenericMatrix<3,3,double> Rz1(Rz1Array);
    QGenericMatrix<3,3,double> Rz2(Rz2Array);
    QGenericMatrix<3,3,double> Rz3(Rz3Array);
    QGenericMatrix<3,3,double> Rtheta_1(Rtheta_1Array);
    QGenericMatrix<3,3,double> Rtheta_2(Rtheta_2Array);
    QGenericMatrix<3,3,double> Rtheta_3(Rtheta_3Array);

    double L_BASE_Array[] = {L_BASE, 0, 0};
    double L_LA_Array[] = {L_LA, 0, 0};
    double L_EE_Array[] = {L_EE, 0, 0};
    QGenericMatrix<1,3,double> L_BASE_Matrix(L_BASE_Array);
    QGenericMatrix<1,3,double> L_LA_Matrix(L_LA_Array);
    QGenericMatrix<1,3,double> L_EE_Matrix(L_EE_Array);

    // base origin to joint base
    QGenericMatrix<1,3,double> O_F1 = Rz1*L_BASE_Matrix;
    QGenericMatrix<1,3,double> O_F2 = Rz2*L_BASE_Matrix;
    QGenericMatrix<1,3,double> O_F3 = Rz3*L_BASE_Matrix;

    // lower link
    QGenericMatrix<1,3,double> F1_E1 = Rz1*Rtheta_1*L_LA_Matrix;
    QGenericMatrix<1,3,double> F2_E2 = Rz2*Rtheta_2*L_LA_Matrix;
    QGenericMatrix<1,3,double> F3_E3 = Rz3*Rtheta_3*L_LA_Matrix;

    // origin to end of lower link
    QGenericMatrix<1,3,double> O_E1 = O_F1 + F1_E1;
    QGenericMatrix<1,3,double> O_E2 = O_F2 + F2_E2;
    QGenericMatrix<1,3,double> O_E3 = O_F3 + F3_E3;

    // end effector to end of second link
    QGenericMatrix<1,3,double> Q_P1 = Rz1*L_EE_Matrix;
    QGenericMatrix<1,3,double> Q_P2 = Rz2*L_EE_Matrix;
    QGenericMatrix<1,3,double> Q_P3 = Rz3*L_EE_Matrix;

    // end effector position multipliable vector
    double eePosArray[] = {eePos[0], eePos[1], eePos[2]};
    QGenericMatrix<1,3,double> eePosMatrix(eePosArray);

    QGenericMatrix<1,3,double> s1 = eePosMatrix + Q_P1 - O_E1;
    QGenericMatrix<1,3,double> s2 = eePosMatrix + Q_P2 - O_E2;
    QGenericMatrix<1,3,double> s3 = eePosMatrix + Q_P3 - O_E3;

    double vec1Array[] = {-L_LA*sin(motorAngles[0]), 0, L_LA*cos(motorAngles[0])};
    double vec2Array[] = {-L_LA*sin(motorAngles[1]), 0, L_LA*cos(motorAngles[1])};
    double vec3Array[] = {-L_LA*sin(motorAngles[2]), 0, L_LA*cos(motorAngles[2])};

    QGenericMatrix<1,3,double> vec1(vec1Array);
    QGenericMatrix<1,3,double> vec2(vec2Array);
    QGenericMatrix<1,3,double> vec3(vec3Array);

    QGenericMatrix<1,3,double> b1 = Rz1*vec1;
    QGenericMatrix<1,3,double> b2 = Rz2*vec2;
    QGenericMatrix<1,3,double> b3 = Rz3*vec3;

    double JxArray[] = {s1(0,0), s1(1,0), s1(2,0),
                        s2(0,0), s2(1,0), s2(2,0),
                        s3(0,0), s3(1,0), s3(2,0)};

    double JthetaArray[] = {(s1.transposed()*b1)(0,0), 0, 0,
                            0, (s2.transposed()*b2)(0,0), 0,
                            0, 0, (s3.transposed()*b3)(0,0)};
    QGenericMatrix<3,3,double> Jx(JxArray);
    QGenericMatrix<3,3,double> Jtheta(JthetaArray);
    return torque;
}


