#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Initialize()
{
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));
    GraphicsTimer.start(20);
}

void MainWindow::UpdateGUIInfo()
{
    Eigen::Vector3d localMotorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    Eigen::Vector3d localJointAngles = p_CommonData->wearableDelta->GetJointAngles();
    Eigen::Vector3d localCartesianPos = p_CommonData->wearableDelta->GetCartesianPos();
    Eigen::Vector3d localDesiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    Eigen::Vector3d localDesiredTorques = p_CommonData->wearableDelta->GetDesiredTorques(localDesiredForce);


    ui->MotorAngleLCDNumber1->display(localMotorAngles[0]*180/PI);
    ui->MotorAngleLCDNumber2->display(localMotorAngles[1]*180/PI);
    ui->MotorAngleLCDNumber3->display(localMotorAngles[2]*180/PI);

    ui->JointAngleLCDNumber1->display(localJointAngles[0]*180/PI);
    ui->JointAngleLCDNumber2->display(localJointAngles[1]*180/PI);
    ui->JointAngleLCDNumber3->display(localJointAngles[2]*180/PI);

    ui->CartesianXLCDNumber->display(localCartesianPos[0]);
    ui->CartesianYLCDNumber->display(localCartesianPos[1]);
    ui->CartesianZLCDNumber->display(localCartesianPos[2]);

    ui->DesiredForceX->display(localDesiredForce[0]);
    ui->DesiredForceY->display(localDesiredForce[1]);
    ui->DesiredForceZ->display(localDesiredForce[2]);

    ui->MotorTorque1->display(localDesiredTorques[0]);
    ui->MotorTorque2->display(localDesiredTorques[1]);
    ui->MotorTorque3->display(localDesiredTorques[2]);

}

void MainWindow::on_CalibratePushButton_clicked()
{
    p_CommonData->wearableDelta->ZeroEncoders();
}

void MainWindow::on_verticalSliderX_valueChanged(int value)
{
    double xSlider = value/100.0;
    double ySlider = this->ui->verticalSliderY->value()/100.0;
    double zSlider = this->ui->verticalSliderZ->value()/100.0;
    Eigen::Vector3d tempDesiredForce(xSlider, ySlider, zSlider);
    p_CommonData->wearableDelta->SetDesiredForce(tempDesiredForce);
}

void MainWindow::on_verticalSliderY_valueChanged(int value)
{
    double xSlider = this->ui->verticalSliderX->value()/100.0;
    double ySlider = value/100.0;
    double zSlider = this->ui->verticalSliderZ->value()/100.0;
    Eigen::Vector3d tempDesiredForce(xSlider, ySlider, zSlider);
    p_CommonData->wearableDelta->SetDesiredForce(tempDesiredForce);
}

void MainWindow::on_verticalSliderZ_valueChanged(int value)
{
    double xSlider = this->ui->verticalSliderX->value()/100.0;
    double ySlider = this->ui->verticalSliderY->value()/100.0;
    double zSlider = value/100.0;
    Eigen::Vector3d tempDesiredForce(xSlider, ySlider, zSlider);
    p_CommonData->wearableDelta->SetDesiredForce(tempDesiredForce);
}
