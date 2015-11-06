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

        //qDebug() << theta*180/PI;
    }
    return jointAngles;
}


