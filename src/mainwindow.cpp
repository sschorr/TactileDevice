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
    ui->MotorAngleLCDNumber1->display(p_CommonData->wearableDelta->GetMotorAngles()[0]);
    ui->MotorAngleLCDNumber2->display(p_CommonData->wearableDelta->GetMotorAngles()[1]);
    ui->MotorAngleLCDNumber3->display(p_CommonData->wearableDelta->GetMotorAngles()[2]);
}
