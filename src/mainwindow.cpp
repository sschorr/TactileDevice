#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string.h>

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
    p_CommonData->currentControlState = idleControl;

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
        p_CommonData->currentControlState = sliderControlMode;
        double xSlider = this->ui->verticalSliderX->value()/25.0;
        double ySlider = this->ui->verticalSliderY->value()/25.0;
        double zSlider = this->ui->verticalSliderZ->value()/25.0+p_CommonData->neutralPos[2];
        Eigen::Vector3d tempDesiredPos(xSlider, ySlider, zSlider);
        p_CommonData->wearableDelta->SetDesiredPos(tempDesiredPos);
    }

    else if(ui->VRControl->isChecked())
    {
        //let haptics thread determine desired position
        p_CommonData->currentControlState = VRControlMode;
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
    localMotorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    localJointAngles = p_CommonData->wearableDelta->GetJointAngles();
    localCartesianPos = p_CommonData->wearableDelta->GetCartesianPos();
    localDesiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    localDesiredMotorTorques = p_CommonData->wearableDelta->CalcDesiredMotorTorques(localDesiredForce);
    localOutputVoltages = p_CommonData->wearableDelta->ReadVoltageOutput();
    localDesiredPos = p_CommonData->wearableDelta->ReadDesiredPos();
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
    ui->trialNo->display(p_CommonData->trialNo);
    ui->pairNo->display(p_CommonData->pairNo);

    switch(p_CommonData->currentExperimentState)
    {
    case idleExperiment:
        ui->directions->setText("No experiment currently running");
        break;
    case frictionTrial:
        if(p_CommonData->pairNo == 1)
        {
            ui->directions->setText("Press 'N' to explore object 2.");
            ui->objectNo->setText("Object 1");
        } else if(p_CommonData->pairNo == 2)
        {
            ui->directions->setText("Which object had higher friction? \n Press '1' or '2', then press 'L' to lockin answer.");
            ui->objectNo->setText("Object 2");
        }
        break;
    case trialBreak:
        ui->directions->setText("Please take a break. \n Press 'N' to continue.");
        ui->selection->setText("Selection:");
    }
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
    p_CommonData->currentControlState = sinControlMode;
    ui->sliderControl->setChecked(false);
    ui->VRControl->setChecked(false);

    p_CommonData->sinStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
    p_CommonData->bandSinAmp = p_CommonData->bandSinAmpDisp;
    p_CommonData->bandSinFreq = p_CommonData->bandSinFreqDisp;
}

void MainWindow::on_startCircle_clicked()
{
    p_CommonData->currentControlState = circControlMode;
    ui->sliderControl->setChecked(false);
    ui->VRControl->setChecked(false);

    p_CommonData->circStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
}

void MainWindow::on_stopRecord_clicked()
{
    p_CommonData->recordFlag = false;
}

void MainWindow::on_setDirectory_clicked()
{    
    bool ok;


    p_CommonData->dir = QFileDialog::getExistingDirectory(0, "Select Directory for file",
                                        "C:/Users/Charm_Stars/Desktop/Dropbox (Stanford CHARM Lab)/Sam Schorr Research Folder/New Tactile Feedback Device/Protocol Creation/Experiments/Friction Exp/Subjects",
                                        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    p_CommonData->fileName = QInputDialog::getText(0, "Input File Name",
                                     "File Name:", QLineEdit::Normal, " ",
                                     &ok);
}

void MainWindow::on_turnOff_clicked()
{
    p_CommonData->currentControlState = idleControl;
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
            p_CommonData->p_tissueCyl->setTransparencyLevel(1.0, true);
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
            p_CommonData->p_tissueCyl->setTransparencyLevel(0.4, true);

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
    if (a_event->key() == Qt::Key_R)
    {
        if(p_CommonData->currentExperimentState == palpationTrial)
        {
            WriteDataToFile();
            p_CommonData->recordFlag = false;
            p_CommonData->trialNo = p_CommonData->trialNo + 1;
            double tissueRad = 0.1; double lumpRad = 0.015;
            double max = tissueRad - lumpRad; double min = 0;
            double angMax = 2*PI; double angMin = 0;
            double rad = ((double) rand()*(max-min)/(double)RAND_MAX+min);
            double ang = ((double) rand()*(angMax-angMin)/(double)RAND_MAX+angMin);
            p_CommonData->p_tissueLump->setLocalPos(rad*cos(ang),rad*sin(ang),-0.000001);
            p_CommonData->recordFlag = true;
            if(p_CommonData->trialNo > 30)
                ui->directions->setText("Experiment Completed, please contact administrator");

        }
    }
    if (a_event->key() == Qt::Key_N)
    {
        if(p_CommonData->currentExperimentState == trialBreak)
        {
            p_CommonData->currentExperimentState = frictionTrial;
            p_CommonData->trialNo = p_CommonData->trialNo + 1;

        } else if(p_CommonData->pairNo == 1)
        {
            p_CommonData->pairNo = 2;
            p_CommonData->p_expFrictionBox->m_material->setRedCrimson();
            double max = 0.03; double min = -0.03;
            double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
            p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
        }
    }
    if (a_event->key() == Qt::Key_Backspace)
    {
        if(p_CommonData->pairNo == 2)
        {
            p_CommonData->pairNo = 1;
            p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
        }

    }

    if (a_event->key() == Qt::Key_1)
    {
        p_CommonData->subjectAnswer = 1;
        ui->selection->setText("Selection: 1");
    }

    if (a_event->key() == Qt::Key_2)
    {
        p_CommonData->subjectAnswer = 2;
        ui->selection->setText("Selection: 2");
    }

    if (a_event->key() == Qt::Key_L)
    {
        if(p_CommonData->currentExperimentState == frictionTrial)
        {
            WriteDataToFile();
        }

        // check if next trial is a break
        QString nextTrialType = p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo + 1)).toStdString().c_str(), "type", NULL /*default*/);
        if (nextTrialType == "break")
        {
            p_CommonData->currentExperimentState = trialBreak;
            p_CommonData->recordFlag = false;
            p_CommonData->debugData.clear();
        }


        p_CommonData->pairNo = 1;
        p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
        p_CommonData->subjectAnswer = 0;
        ui->selection->setText("Selection:");
        double max = 0.03; double min = -0.03;
        double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
        p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
        p_CommonData->trialNo = p_CommonData->trialNo + 1;
    }
}

void MainWindow::WriteDataToFile()
{
    p_CommonData->recordFlag = false;

    char buffer[33];
    itoa(p_CommonData->trialNo,buffer,10);

    //write data to file when we are done
    std::ofstream file;
    file.open(p_CommonData->dir.toStdString() + "/" + p_CommonData->fileName.toStdString() + buffer + ".txt");
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
        << p_CommonData->debugData[i].magTrackerPos0.x() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos0.y() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos0.z() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.x() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.y() << "," << " "
        << p_CommonData->debugData[i].magTrackerPos1.z() << "," << " "
        << p_CommonData->debugData[i].accelSignal.x() << "," << " "
        << p_CommonData->debugData[i].accelSignal.y() << "," << " "
        << p_CommonData->debugData[i].accelSignal.z() << "," << " "
        << p_CommonData->debugData[i].tactileFeedback << "," << " "
        << p_CommonData->debugData[i].referenceFirst << "," << " "
        << p_CommonData->debugData[i].pairNo << "," << " "
        << p_CommonData->debugData[i].referenceFriction << "," << " "
        << p_CommonData->debugData[i].comparisonFriction << "," << " "
        << p_CommonData->debugData[i].subjectAnswer << "," << " "
        << p_CommonData->debugData[i].lumpLocation.x() << "," << " "
        << p_CommonData->debugData[i].lumpLocation.y() << "," << " "
        << std::endl;
    }
    file.close();
    p_CommonData->debugData.clear();
    p_CommonData->recordFlag = true;
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

void MainWindow::on_dynamicEnvironment_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = dynamicBodies;
}

void MainWindow::on_paperEnvironment_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = paperEnvironment;
}

void MainWindow::on_palpExp_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = experimentPalpation;
}

void MainWindow::on_loadProtocol_clicked()
{
    //Open dialog box to get protocol file and save into variable
    QString temp = QFileDialog::getOpenFileName();
    p_CommonData->frictionProtocolLocation = temp;
    p_CommonData->frictionProtocolFile.LoadFile(temp.toStdString().c_str());
    qDebug() << p_CommonData->frictionProtocolLocation;
}

void MainWindow::on_loadProtocol_2_clicked()
{
    //Open dialog box to get protocol file and save into variable
    QString temp = QFileDialog::getOpenFileName();
    p_CommonData->palpationProtocolLocation = temp;
    p_CommonData->palpationProtocolFile.LoadFile(temp.toStdString().c_str());
    qDebug() << p_CommonData->palpationProtocolLocation;
}

void MainWindow::on_startExperiment_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = frictionTrial;
    p_CommonData->currentEnvironmentState = experimentFriction;
    p_CommonData->currentControlState = VRControlMode;
    p_CommonData->pairNo = 1;
    p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
}

void MainWindow::on_startExperiment_2_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = palpationTrial;
    p_CommonData->currentEnvironmentState = experimentPalpation;
    p_CommonData->currentControlState = VRControlMode;

}

void MainWindow::on_setNeutral_clicked()
{
    p_CommonData->neutralPos[2] = localCartesianPos[2];
    on_ZeroSliders_clicked();
}

void MainWindow::on_setTrial_clicked()
{
    bool ok;
    QString TrialNoString = QInputDialog::getText(0, "Input Trial No",
                                         "Trial #:", QLineEdit::Normal, " ",
                                         &ok);
    p_CommonData->trialNo = TrialNoString.toInt();
}




