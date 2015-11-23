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
    p_CommonData->hapticsThreadActive = false;

    //write debugging data to file when we are done
    std::ofstream file;
    file.open("C:/Users/Charm_Stars/Desktop/Sam Projects/Sam GitProjects/DataWrite/testData3.txt");
    for (int i=0; i < p_CommonData->debugData.size(); i++)

    {
        //[0] is distal finger, [1] is toward middle finger, [2] is away from finger pad
        file << p_CommonData->debugData[i].time << "," << " "
             << p_CommonData->debugData[i].pos[0] << "," << " "
             << p_CommonData->debugData[i].pos[1] << "," << " "
             << p_CommonData->debugData[i].pos[2] << "," << " "
             << p_CommonData->debugData[i].desiredPos[0] << "," << " "
             << p_CommonData->debugData[i].desiredPos[1] << "," << " "
             << p_CommonData->debugData[i].desiredPos[2] << "," << " "
             << p_CommonData->debugData[i].desiredForce[0] << "," << " "
             << p_CommonData->debugData[i].desiredForce[1] << "," << " "
             << p_CommonData->debugData[i].desiredForce[2] << "," << " "
             << p_CommonData->debugData[i].motorAngles[0] << "," << " "
             << p_CommonData->debugData[i].motorAngles[1] << "," << " "
             << p_CommonData->debugData[i].motorAngles[2] << "," << " "
             << p_CommonData->debugData[i].jointAngles[0] << "," << " "
             << p_CommonData->debugData[i].jointAngles[1] << "," << " "
             << p_CommonData->debugData[i].jointAngles[2] << "," << " "
             << p_CommonData->debugData[i].motorTorque[0] << "," << " "
             << p_CommonData->debugData[i].motorTorque[1] << "," << " "
             << p_CommonData->debugData[i].motorTorque[2] << "," << " "
             << p_CommonData->debugData[i].voltageOut[0] << "," << " "
             << p_CommonData->debugData[i].voltageOut[1] << "," << " "
             << p_CommonData->debugData[i].voltageOut[2] << "," << " "

             << std::endl;
    }
    file.close();
    delete ui;
}

void MainWindow::Initialize()
{
    connect(this->ui->verticalSliderX, SIGNAL(valueChanged(int)), this, SLOT(on_GUI_changed()));
    connect(this->ui->verticalSliderY, SIGNAL(valueChanged(int)), this, SLOT(on_GUI_changed()));
    connect(this->ui->verticalSliderZ, SIGNAL(valueChanged(int)), this, SLOT(on_GUI_changed()));
    connect(this->ui->radioButtonPos, SIGNAL(clicked()), this, SLOT(on_GUI_changed()));
    connect(this->ui->radioButtonForce, SIGNAL(clicked()), this, SLOT(on_GUI_changed()));
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));

    GraphicsTimer.start(20);
    ui->radioButtonPos->click();
}

void MainWindow::on_GUI_changed()
{
    if(ui->radioButtonForce->isChecked())
    {
        p_CommonData->posControlMode = false;
        p_CommonData->forceControlMode = true;
        double xSlider = this->ui->verticalSliderX->value()/10.0;
        double ySlider = this->ui->verticalSliderY->value()/10.0;
        double zSlider = this->ui->verticalSliderZ->value()/10.0;
        Eigen::Vector3d tempDesiredForce(xSlider, ySlider, zSlider);
        p_CommonData->wearableDelta->SetDesiredForce(tempDesiredForce);
    }
    else if(ui->radioButtonPos->isChecked())
    {
        p_CommonData->forceControlMode = false;
        p_CommonData->posControlMode = true;
        double xSlider = this->ui->verticalSliderX->value()/25.0;
        double ySlider = this->ui->verticalSliderY->value()/25.0;
        double zSlider = this->ui->verticalSliderZ->value()/25.0+12.73;
        Eigen::Vector3d tempDesiredPos(xSlider, ySlider, zSlider);
        p_CommonData->wearableDelta->SetDesiredPos(tempDesiredPos);
    }
    UpdateGUIInfo();
}

void MainWindow::UpdateGUIInfo()
{
    Eigen::Vector3d localMotorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    Eigen::Vector3d localJointAngles = p_CommonData->wearableDelta->GetJointAngles();
    Eigen::Vector3d localCartesianPos = p_CommonData->wearableDelta->GetCartesianPos();
    Eigen::Vector3d localDesiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    Eigen::Vector3d localDesiredMotorTorques = p_CommonData->wearableDelta->CalcDesiredMotorTorques(localDesiredForce);
    Eigen::Vector3d localOutputVoltages = p_CommonData->wearableDelta->ReadVoltageOutput();
    Eigen::Vector3d localDesiredPos = p_CommonData->wearableDelta->ReadDesiredPos();

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

    ui->MotorTorque1->display(localDesiredMotorTorques[0]);
    ui->MotorTorque2->display(localDesiredMotorTorques[1]);
    ui->MotorTorque3->display(localDesiredMotorTorques[2]);

    ui->VoltageLCD1->display((double)(localOutputVoltages[0]));
    ui->VoltageLCD2->display((double)(localOutputVoltages[1]));
    ui->VoltageLCD3->display((double)(localOutputVoltages[2]));

    ui->DesX->display(localDesiredPos[0]);
    ui->DesY->display(localDesiredPos[1]);
    ui->DesZ->display(localDesiredPos[2]);

    ui->lcdNumberHapticRate->display(p_CommonData->hapticRateEstimate);
}

void MainWindow::on_CalibratePushButton_clicked()
{
    p_CommonData->wearableDelta->ZeroEncoders();

    on_GUI_changed();
}

void MainWindow::on_ZeroSliders_clicked()
{
    ui->verticalSliderX->blockSignals(true);
    ui->verticalSliderY->blockSignals(true);
    ui->verticalSliderZ->blockSignals(true);
    ui->verticalSliderX->setValue(0);
    ui->verticalSliderY->setValue(0);
    ui->verticalSliderZ->setValue(0);
    ui->verticalSliderX->blockSignals(false);
    ui->verticalSliderY->blockSignals(false);
    ui->verticalSliderZ->blockSignals(false);

    on_GUI_changed();
}
