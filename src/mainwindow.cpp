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
    delete ui;
}

void MainWindow::Initialize()
{
    // Set our current state
    p_CommonData->currentState = idle;

    // Initialize shared memory for OpenGL widget
    ui->DisplayWidget->p_CommonData = p_CommonData;

    connect(this->ui->verticalSliderX, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderY, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderZ, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->sliderControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->VRControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));

    GraphicsTimer.start(20);
    UpdateGUIInfo();
}

void MainWindow::onGUIchanged()
{
    if(ui->sliderControl->isChecked())
    {
        p_CommonData->currentState = sliderControlMode;
        double xSlider = this->ui->verticalSliderX->value()/25.0;
        double ySlider = this->ui->verticalSliderY->value()/25.0;
        double zSlider = this->ui->verticalSliderZ->value()/25.0+12.73;
        Eigen::Vector3d tempDesiredPos(xSlider, ySlider, zSlider);
        p_CommonData->wearableDelta->SetDesiredPos(tempDesiredPos);
    }

    else if(ui->VRControl->isChecked())
    {
        //let haptics thread determine desired position
        p_CommonData->currentState = VRControlMode;
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
    onGUIchanged();
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
    onGUIchanged();
}

void MainWindow::on_startSin_clicked()
{
    p_CommonData->currentState = sinControlMode;
    ui->sliderControl->setChecked(false);
    ui->VRControl->setChecked(false);

    p_CommonData->sinStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
}

void MainWindow::on_stopRecord_clicked()
{
    p_CommonData->recordFlag = false;

    QString dir;
    QString fileName;
    bool ok;


    dir = QFileDialog::getExistingDirectory(0, "Select Directory for file",
                                        "C:/Users/Charm_Stars/Dropbox(Stanford CHARM Lab)/Sam Schorr Research Folder/New Tactile Feedback Device/Data/",
                                        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    fileName = QInputDialog::getText(0, "Input File Name",
                                     "File Name:", QLineEdit::Normal, " ",
                                     &ok);
    //write debugging data to file when we are done
    std::ofstream file;
    file.open(dir.toStdString() + "/" + fileName.toStdString());
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
             << p_CommonData->debugData[i].magTrackerPos.x() << "," << " "
             << p_CommonData->debugData[i].magTrackerPos.y() << "," << " "
             << p_CommonData->debugData[i].magTrackerPos.z() << "," << " "
             << p_CommonData->debugData[i].accelSignal.x() << "," << " "
             << p_CommonData->debugData[i].accelSignal.y() << "," << " "
             << p_CommonData->debugData[i].accelSignal.z() << "," << " "
             << std::endl;
    }
    file.close();
}

void MainWindow::on_startRecord_clicked()
{
    p_CommonData->recordFlag = true;
}
