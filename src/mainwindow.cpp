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
    ui->MotorAngleLCDNumber1->display(p_CommonData->wearableDelta->GetMotorAngles()[0]*180/PI);
    ui->JointAngleLCDNumber1->display(p_CommonData->wearableDelta->GetJointAngles()[0]*180/PI);

    ui->MotorAngleLCDNumber2->display(p_CommonData->wearableDelta->GetMotorAngles()[1]*180/PI);
    ui->JointAngleLCDNumber2->display(p_CommonData->wearableDelta->GetJointAngles()[1]*180/PI);

    ui->MotorAngleLCDNumber3->display(p_CommonData->wearableDelta->GetMotorAngles()[2]*180/PI);
    ui->JointAngleLCDNumber3->display(p_CommonData->wearableDelta->GetJointAngles()[2]*180/PI);

    ui->CartesianXLCDNumber->display(p_CommonData->wearableDelta->GetCartesianPos()[0]);
    ui->CartesianYLCDNumber->display(p_CommonData->wearableDelta->GetCartesianPos()[1]);
    ui->CartesianZLCDNumber->display(p_CommonData->wearableDelta->GetCartesianPos()[2]);

    ui->DesiredForceX->display(p_CommonData->wearableDelta->ReadDesiredForce()[0]);
    ui->DesiredForceY->display(p_CommonData->wearableDelta->ReadDesiredForce()[1]);
    ui->DesiredForceZ->display(p_CommonData->wearableDelta->ReadDesiredForce()[2]);

    ui->MotorTorque1->display((p_CommonData->wearableDelta->GetDesiredTorques(p_CommonData->wearableDelta->ReadDesiredForce())[0]));
    ui->MotorTorque2->display((p_CommonData->wearableDelta->GetDesiredTorques(p_CommonData->wearableDelta->ReadDesiredForce())[1]));
    ui->MotorTorque3->display((p_CommonData->wearableDelta->GetDesiredTorques(p_CommonData->wearableDelta->ReadDesiredForce())[2]));

}

void MainWindow::on_CalibratePushButton_clicked()
{
    p_CommonData->wearableDelta->ZeroEncoders();
}

void MainWindow::on_verticalSliderX_valueChanged(int value)
{
    p_CommonData->GUI_desiredX = double(value)/100.0;
}

void MainWindow::on_verticalSliderY_valueChanged(int value)
{
    p_CommonData->GUI_desiredY = double(value)/100.0;
}

void MainWindow::on_verticalSliderZ_valueChanged(int value)
{
    p_CommonData->GUI_desiredZ = double(value)/100.0;
}
