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
    p_CommonData->currentEnvironmentState = none;

    // Initialize shared memory for OpenGL widget
    ui->DisplayWidget->p_CommonData = p_CommonData;    

    connect(this->ui->verticalSliderX, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderY, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderZ, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->KpSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->KdSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->bandwidthAmpSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->bandwidthFreqSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));

    connect(this->ui->sliderControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->VRControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));

    // init slider values
    this->ui->KpSlider->setValue(50);
    this->ui->KdSlider->setValue(10);
    this->ui->bandwidthAmpSlider->setValue(50);
    this->ui->bandwidthFreqSlider->setValue(10);

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

    double KpSlider = this->ui->KpSlider->value()/2.0;
    double KdSlider = this->ui->KdSlider->value()/50.0;
    double bandwidthAmp = this->ui->bandwidthAmpSlider->value()/20.0;
    double bandwidthFreq = this->ui->bandwidthFreqSlider->value()/10.0;

    p_CommonData->Kp = KpSlider;
    p_CommonData->Kd = KdSlider;
    p_CommonData->bandSinAmpDisp = bandwidthAmp;
    p_CommonData->bandSinFreqDisp = bandwidthFreq;

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
    ui->lcdBandAmp->display(p_CommonData->bandSinAmpDisp);
    ui->lcdBandFreq->display(p_CommonData->bandSinFreqDisp);
    ui->lcdKp->display(p_CommonData->Kp);
    ui->lcdKd->display(p_CommonData->Kd);
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
    p_CommonData->bandSinAmp = p_CommonData->bandSinAmpDisp;
    p_CommonData->bandSinFreq = p_CommonData->bandSinFreqDisp;
}

void MainWindow::on_stopRecord_clicked()
{
    p_CommonData->recordFlag = false;
}

void MainWindow::on_setDirectory_clicked()
{    
    bool ok;


    p_CommonData->dir = QFileDialog::getExistingDirectory(0, "Select Directory for file",
                                        "C:/Users/Charm_Stars/Dropbox(Stanford CHARM Lab)/Sam Schorr Research Folder/New Tactile Feedback Device/Data/",
                                        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    p_CommonData->fileName = QInputDialog::getText(0, "Input File Name",
                                     "File Name:", QLineEdit::Normal, " ",
                                     &ok);
}

void MainWindow::on_turnOff_clicked()
{
    p_CommonData->currentState = idle;
    ui->sliderControl->setAutoExclusive(false);
    ui->VRControl->setAutoExclusive(false);

    ui->sliderControl->setChecked(false);
    ui->VRControl->setChecked(false);

    ui->sliderControl->setAutoExclusive(true);
    ui->VRControl->setAutoExclusive(true);
}

void MainWindow::keyPressEvent(QKeyEvent *a_event)
{
    if (a_event->key() == Qt::Key_T)
    {
        if (p_CommonData->m_flagTissueTransparent == true)
        {
            p_CommonData->m_flagTissueTransparent = false;
            p_CommonData->p_tissueOne->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(1.0, true);
        }
        else
        {
            p_CommonData->m_flagTissueTransparent = true;
            p_CommonData->p_tissueOne->setTransparencyLevel(0.0, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(0.0, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(0.0, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(0.0, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(0.0, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(0.0, true);
        }
    }

    double degInc = 5.0;
    double radInc = 0.05;
    if (a_event->key() == Qt::Key_W)
    {
        p_CommonData->polar = p_CommonData->polar + degInc;
    }
    if (a_event->key() == Qt::Key_S)
    {
        p_CommonData->polar = p_CommonData->polar - degInc;
    }
    if (a_event->key() == Qt::Key_A)
    {
        p_CommonData->azimuth = p_CommonData->azimuth + degInc;
    }
    if (a_event->key() == Qt::Key_D)
    {
        p_CommonData->azimuth = p_CommonData->azimuth - degInc;
    }
    if (a_event->key() == Qt::Key_Q)
    {
        p_CommonData->camRadius = p_CommonData->camRadius - radInc;
    }
    if (a_event->key() == Qt::Key_E)
    {
        p_CommonData->camRadius = p_CommonData->camRadius + radInc;
    }



}

void MainWindow::on_palpationButton_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = palpation;
}

void MainWindow::on_frictionButton_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = friction;
}

void MainWindow::on_humpButton_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = hump;
}

void MainWindow::on_hoopHumpButton_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = hoopHump;
}
