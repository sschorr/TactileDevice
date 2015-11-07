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

    QVector<double> test(3);
    test[0] = 0; test[1] = 0; test[1] = 1;

    p_CommonData->wearableDelta->GetDesiredTorques(test);

}

void MainWindow::on_CalibratePushButton_clicked()
{
    p_CommonData->wearableDelta->ZeroEncoders();
}
