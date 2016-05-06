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

    connect(this->ui->OneUp, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->OneDown, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->TwoUp, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->TwoDown, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->ThreeUp, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->ThreeDown, SIGNAL(clicked()), this, SLOT(onGUIchanged()));

    connect(this->ui->sliderControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->VRControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));

    // init slider values
    this->ui->KpSlider->setValue(60);
    this->ui->KdSlider->setValue(30);

    this->ui->bandwidthAmpSlider->setValue(70);
    this->ui->bandwidthFreqSlider->setValue(10);

    p_CommonData->indicatorRot = 0;
    p_CommonData->tissueRot = 0;

    p_CommonData->desJointInits = p_CommonData->wearableDelta->GetJointAngles();

    GraphicsTimer.start(20);
    UpdateGUIInfo();
}

void MainWindow::onGUIchanged()
{
    if(ui->initJoints->isChecked())
    {
        p_CommonData->currentControlState = initCalibControl;
    }
    if(ui->sliderControl->isChecked())
    {
        p_CommonData->currentControlState = sliderControlMode;
        double xSlider = this->ui->verticalSliderX->value()/22.2;
        double ySlider = this->ui->verticalSliderY->value()/22.2;
        double zSlider = this->ui->verticalSliderZ->value()/22.2+p_CommonData->neutralPos[2];
        Eigen::Vector3d tempDesiredPos(xSlider, ySlider, zSlider);
        p_CommonData->wearableDelta->SetDesiredPos(tempDesiredPos);
    }

    else if(ui->VRControl->isChecked())
    {
        //let haptics thread determine desired position
        p_CommonData->currentControlState = VRControlMode;
    }



    double KpSlider = this->ui->KpSlider->value()*20.0;
    double KdSlider = this->ui->KdSlider->value()/30.0;

    double bandwidthAmp = this->ui->bandwidthAmpSlider->value()/20.0;
    double bandwidthFreq = this->ui->bandwidthFreqSlider->value()/3;

    p_CommonData->jointKp = KpSlider;
    p_CommonData->jointKd = KdSlider;
    p_CommonData->bandSinAmpDisp = bandwidthAmp;
    p_CommonData->bandSinFreqDisp = bandwidthFreq;

    ui->directions->setText("No experiment currently running");

    UpdateGUIInfo();
}

void MainWindow::UpdateGUIInfo()
{
    localMotorAngles = p_CommonData->wearableDelta->GetMotorAngles();
    localJointAngles = p_CommonData->wearableDelta->GetJointAngles();
    localCartesianPos = p_CommonData->wearableDelta->GetCartesianPos();
    localDesiredForce = p_CommonData->wearableDelta->ReadDesiredForce();
    localOutputVoltages = p_CommonData->wearableDelta->ReadVoltageOutput();
    localDesiredPos = p_CommonData->wearableDelta->ReadDesiredPos();
    localDesiredJointAngle = p_CommonData->desJointInits;//p_CommonData->wearableDelta->CalcInverseKinJoint();

    ui->MotorAngleLCDNumber1->display(localMotorAngles[0]*180/PI);
    ui->MotorAngleLCDNumber2->display(localMotorAngles[1]*180/PI);
    ui->MotorAngleLCDNumber3->display(localMotorAngles[2]*180/PI);
    ui->JointAngleLCDNumber1->display(localJointAngles[0]*180/PI);
    ui->JointAngleLCDNumber2->display(localJointAngles[1]*180/PI);
    ui->JointAngleLCDNumber3->display(localJointAngles[2]*180/PI);
    ui->CartesianXLCDNumber->display(localCartesianPos[0]);
    ui->CartesianYLCDNumber->display(localCartesianPos[1]);
    ui->CartesianZLCDNumber->display(localCartesianPos[2]);
    ui->desAngle1->display(localDesiredJointAngle[0]*180/PI);
    ui->desAngle2->display(localDesiredJointAngle[1]*180/PI);
    ui->desAngle3->display(localDesiredJointAngle[2]*180/PI);
    ui->DesiredForceX->display(localDesiredForce[0]);
    ui->DesiredForceY->display(localDesiredForce[1]);
    ui->DesiredForceZ->display(localDesiredForce[2]);
    ui->VoltageLCD1->display((double)(localOutputVoltages[0]));
    ui->VoltageLCD2->display((double)(localOutputVoltages[1]));
    ui->VoltageLCD3->display((double)(localOutputVoltages[2]));
    ui->DesX->display(localDesiredPos[0]);
    ui->DesY->display(localDesiredPos[1]);
    ui->DesZ->display(localDesiredPos[2]);
    ui->lcdNumberHapticRate->display(p_CommonData->hapticRateEstimate);
    ui->lcdBandAmp->display(p_CommonData->bandSinAmpDisp);
    ui->lcdBandFreq->display(p_CommonData->bandSinFreqDisp);
    ui->lcdKp->display(p_CommonData->jointKp);
    ui->lcdKd->display(p_CommonData->jointKd);
    ui->trialNo->display(p_CommonData->trialNo);
    ui->pairNo->display(p_CommonData->pairNo);

    //qDebug() << "indicator Rot: " << p_CommonData->indicatorRot << "tissue rot: " << p_CommonData->tissueRot;

    switch(p_CommonData->currentExperimentState)
    {
    case idleExperiment:
        ///ui->directions->setText("No experiment currently running");
        break;

    case frictionTrial:
        if(p_CommonData->pairNo == 1)
        {
            ui->directions->setText("Press 'Q' to toggle between surfaces \n'R' to lock in answer.");
            ui->objectNo->setText("Current Surface: 1");
        } else if(p_CommonData->pairNo == 2)
        {
            ui->directions->setText("Press 'Q' to toggle between surfaces \n'R' to lock in answer.");
            ui->objectNo->setText("Current Surface: 2");
        }
        break;

    case palpationTrial:
        if (p_CommonData->trialNo < 30)
        {
            ui->directions->setText("Press 'R' to record answer and move to next trial");
        }
        break;

    case palpationLineTrial:
        ui->directions->setText("Press 'Z' or 'X' to rotate answer.  Press 'R' to lock in choice");
        break;

    case end:
        ui->directions->setText("Experiment over, please contact administrator");
        break;

    case palpationLineBreak:
        ui->directions->setText("Please take a break.  Press 'R' to continue");
        break;

    case trialBreak:
        ui->directions->setText("Please take a break. \nPress 'N' to continue.");
        ui->selection->setText("Stiffer Object:");
        break;
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
    p_CommonData->debugData.clear();
    p_CommonData->currentControlState = sinControlMode;
    ui->sliderControl->setChecked(false);
    ui->VRControl->setChecked(false);

    p_CommonData->sinStartTime = p_CommonData->overallClock.getCurrentTimeSeconds();
    p_CommonData->bandSinAmp = p_CommonData->bandSinAmpDisp;
    p_CommonData->bandSinFreq = p_CommonData->bandSinFreqDisp;
}

void MainWindow::on_startCircle_clicked()
{
    p_CommonData->debugData.clear();
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
                                        "C:/Users/Charm_Stars/Desktop/Dropbox (Stanford CHARM Lab)/Sam Schorr Research Folder/New Tactile Feedback Device/Protocol Creation/Experiments/PalpationLine Exp/Subjects",
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
            p_CommonData->p_tissueSeven->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueEight->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueNine->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueTen->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueEleven->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueTwelve->setTransparencyLevel(1, true);
            p_CommonData->p_indicator->setTransparencyLevel(0, true);

            p_CommonData->p_tissueCyl->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueLump->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueLumpCenter->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueLumpCenter1->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueLumpCenter2->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueLumpCenter3->setTransparencyLevel(1.0, true);
            p_CommonData->p_tissueBox->setTransparencyLevel(1.0, true);
        }
        else
        {
            p_CommonData->m_flagTissueTransparent = true;
            p_CommonData->p_tissueOne->setTransparencyLevel(0.1, true);
            p_CommonData->p_tissueTwo->setTransparencyLevel(0.2, true);
            p_CommonData->p_tissueThree->setTransparencyLevel(0.3, true);
            p_CommonData->p_tissueFour->setTransparencyLevel(0.4, true);
            p_CommonData->p_tissueFive->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueSix->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueSeven->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueEight->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueNine->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueTen->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueEleven->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueTwelve->setTransparencyLevel(0.5, true);
            p_CommonData->p_indicator->setTransparencyLevel(1, true);

            p_CommonData->p_tissueCyl->setTransparencyLevel(0.2, true);
            p_CommonData->p_tissueBox->setTransparencyLevel(0.2, true);
            p_CommonData->p_tissueLump->setTransparencyLevel(0.4, true);
            p_CommonData->p_tissueLumpCenter->setTransparencyLevel(0.5, true);
            p_CommonData->p_tissueLumpCenter1->setTransparencyLevel(0.65, true);
            p_CommonData->p_tissueLumpCenter2->setTransparencyLevel(0.8, true);
            p_CommonData->p_tissueLumpCenter3->setTransparencyLevel(1.0, true);
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
    if (a_event->key() == Qt::Key_C)
    {
        p_CommonData->camRadius = p_CommonData->camRadius - radInc;
    }
    if (a_event->key() == Qt::Key_V)
    {
        p_CommonData->camRadius = p_CommonData->camRadius + radInc;
    }
    if (a_event->key() == Qt::Key_R)
    {
        // friction experiment
        if(p_CommonData->currentExperimentState == frictionTrial)
        {
            if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
            {
                if(p_CommonData->currentExperimentState == frictionTrial)
                {
                    if(p_CommonData->subjectAnswer == 1 || p_CommonData->subjectAnswer == 2)
                    {
                        WriteDataToFile();

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
                        ui->selection->setText("Higher Friction:");
                        double max = 0.03; double min = -0.03;
                        double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                        p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
                        p_CommonData->trialNo = p_CommonData->trialNo + 1;
                        p_CommonData->recordFlag = true;
                    }
                }
            }
        }


        // palpation Line experiment
        if(p_CommonData->currentExperimentState == palpationLineTrial)
        {
            // check that we are not currently embedded in tissue
            if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
            {
                p_CommonData->currentExperimentState = palpationLineWritingToFile;
                p_CommonData->recordFlag = false;
                WriteDataToFile();
                p_CommonData->p_tissueOne->setTransparencyLevel(0.25, true);
                p_CommonData->p_tissueTwo->setTransparencyLevel(0.3, true);
                p_CommonData->p_tissueThree->setTransparencyLevel(0.35, true);
                p_CommonData->p_tissueFour->setTransparencyLevel(0.4, true);
                p_CommonData->p_tissueFive->setTransparencyLevel(0.45, true);
                p_CommonData->p_tissueSix->setTransparencyLevel(0.5, true);
                p_CommonData->p_tissueSeven->setTransparencyLevel(0.55, true);
                p_CommonData->p_tissueEight->setTransparencyLevel(0.6, true);
                p_CommonData->p_tissueNine->setTransparencyLevel(0.65, true);
                p_CommonData->p_tissueTen->setTransparencyLevel(0.7, true);
                p_CommonData->p_tissueEleven->setTransparencyLevel(0.75, true);
                p_CommonData->p_tissueTwelve->setTransparencyLevel(0.8, true);
                p_CommonData->p_indicator->setTransparencyLevel(1, true);

                p_CommonData->palpPostTrialClock.reset();
                p_CommonData->palpPostTrialClock.start();
            }
        }

        else if(p_CommonData->currentExperimentState == palpationLineBreak)
        {
            if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
            {
                p_CommonData->currentExperimentState = palpationLineWritingToFile;
                p_CommonData->recordFlag = false;

                p_CommonData->palpPostTrialClock.reset();
                p_CommonData->palpPostTrialClock.start();
            }
        }

        // standard palpation experiment
        if(p_CommonData->currentExperimentState == palpationTrial)
        {
            if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
            {
                p_CommonData->currentExperimentState = idleExperiment;
                WriteDataToFile();
                if(p_CommonData->trialNo > 34)
                    ui->directions->setText("Experiment Completed, please contact administrator");
                p_CommonData->p_tissueCyl->setTransparencyLevel(0.2, true);
                p_CommonData->p_tissueBox->setTransparencyLevel(0.2, true);
                p_CommonData->p_tissueLump->setTransparencyLevel(0.4, true);
                p_CommonData->p_tissueLumpCenter->setTransparencyLevel(0.5, true);
                p_CommonData->p_tissueLumpCenter1->setTransparencyLevel(0.65, true);
                p_CommonData->p_tissueLumpCenter2->setTransparencyLevel(0.8, true);
                p_CommonData->p_tissueLumpCenter3->setTransparencyLevel(1.0, true);

                p_CommonData->palpPostTrialClock.reset();
                p_CommonData->palpPostTrialClock.start();
            }
        }
    }

    if (a_event->key() == Qt::Key_Backspace)
    {
        if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
        {
            if(p_CommonData->pairNo == 2)
            {
                p_CommonData->pairNo = 1;
                p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
                double max = 0.03; double min = -0.01;
                double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
            }
        }
    }

    if (a_event->key() == Qt::Key_1)
    {
        p_CommonData->subjectAnswer = 1;
        ui->selection->setText("Higher Friction: 1 (blue)");
    }

    if (a_event->key() == Qt::Key_2)
    {
        p_CommonData->subjectAnswer = 2;
        ui->selection->setText("Higher Friction: 2 (red)");
    }

    if (a_event->key() == Qt::Key_Q)
    {
        if(p_CommonData->currentExperimentState == trialBreak)
        {
            p_CommonData->currentExperimentState = frictionTrial;
            p_CommonData->trialNo = p_CommonData->trialNo + 1;
        }

        else if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
        {
            if(p_CommonData->pairNo == 1)
            {
                p_CommonData->pairNo = 2;
                p_CommonData->p_expFrictionBox->m_material->setRedCrimson();
                double max = 0.03; double min = -0.01;
                double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
            }

            else if(p_CommonData->pairNo == 2)
            {
                p_CommonData->pairNo = 1;
                p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
                double max = 0.03; double min = -0.01;
                double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
            }
        }
    }

    if (a_event->key() == Qt::Key_L)
    {

    }

    double angle = 10.0;
    if (a_event->key() == Qt::Key_Z)
    {
        rotateTissueLineDisp(-angle);
        p_CommonData->p_indicator->setTransparencyLevel(1, true);
    }
    if (a_event->key() == Qt::Key_X)
    {
        rotateTissueLineDisp(angle);
        p_CommonData->p_indicator->setTransparencyLevel(1, true);
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
        << p_CommonData->debugData[i].VRInteractionForce[0] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForce[1] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForce[2] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[0] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[1] << "," << " "
        << p_CommonData->debugData[i].VRInteractionForceGlobal[2] << "," << " "
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
        << p_CommonData->debugData[i].lineAngle << "," << " "
        << p_CommonData->debugData[i].lineAngleTruth << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(0,2) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(1,2) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,0) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,1) << "," << " "
        << p_CommonData->debugData[i].deviceRotation(2,2) << "," << " "
        << std::endl;
    }
    file.close();
    p_CommonData->debugData.clear();
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
    int error = p_CommonData->frictionProtocolFile.LoadFile(temp.toStdString().c_str());
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

void MainWindow::on_loadProtocol_3_clicked()
{
    //Open dialog box to get protocol file and save into variable
    QString temp = QFileDialog::getOpenFileName();
    p_CommonData->palpationLineProtocolLocation = temp;
    p_CommonData->palpationLineProtocolFile.LoadFile(temp.toStdString().c_str());
    qDebug() << p_CommonData->palpationLineProtocolLocation;
}

void MainWindow::on_startExperiment_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = frictionTrial;
    p_CommonData->currentEnvironmentState = experimentFriction;
    p_CommonData->currentControlState = VRControlMode;
    p_CommonData->pairNo = 1;
    p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
    double degInc = 5.0;
    double radInc = 0.05;
    p_CommonData->polar = p_CommonData->polar - 5*degInc;
    p_CommonData->camRadius = p_CommonData->camRadius + radInc;
}

void MainWindow::on_startExperiment_2_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = palpationTrial;
    p_CommonData->currentEnvironmentState = experimentPalpation;
    p_CommonData->currentControlState = VRControlMode;
    p_CommonData->recordFlag = true;

    p_CommonData->p_tissueCyl->setTransparencyLevel(0.2, true);
    p_CommonData->p_tissueBox->setTransparencyLevel(0.2, true);
    p_CommonData->p_tissueLump->setTransparencyLevel(0.4, true);
    p_CommonData->p_tissueLumpCenter->setTransparencyLevel(0.5, true);
    p_CommonData->p_tissueLumpCenter1->setTransparencyLevel(0.65, true);
    p_CommonData->p_tissueLumpCenter2->setTransparencyLevel(0.8, true);
    p_CommonData->p_tissueLumpCenter3->setTransparencyLevel(1.0, true);
}

void MainWindow::on_startExperiment_3_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = palpationLineTrial;
    p_CommonData->currentEnvironmentState = experimentPalpationLine;
    p_CommonData->currentControlState = VRControlMode;

    // check that we are not currently embedded in tissue
    if(!(localDesiredPos[2] < p_CommonData->neutralPos[2]))
    {
        p_CommonData->p_tissueOne->setTransparencyLevel(0.25, true);
        p_CommonData->p_tissueTwo->setTransparencyLevel(0.3, true);
        p_CommonData->p_tissueThree->setTransparencyLevel(0.35, true);
        p_CommonData->p_tissueFour->setTransparencyLevel(0.4, true);
        p_CommonData->p_tissueFive->setTransparencyLevel(0.45, true);
        p_CommonData->p_tissueSix->setTransparencyLevel(0.5, true);
        p_CommonData->p_tissueSeven->setTransparencyLevel(0.55, true);
        p_CommonData->p_tissueEight->setTransparencyLevel(0.6, true);
        p_CommonData->p_tissueNine->setTransparencyLevel(0.65, true);
        p_CommonData->p_tissueTen->setTransparencyLevel(0.7, true);
        p_CommonData->p_tissueEleven->setTransparencyLevel(0.75, true);
        p_CommonData->p_tissueTwelve->setTransparencyLevel(0.8, true);
        p_CommonData->p_indicator->setTransparencyLevel(1, true);
        p_CommonData->trialNo = p_CommonData->trialNo - 1;

        p_CommonData->palpPostTrialClock.reset();
        p_CommonData->palpPostTrialClock.start();
    }
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

void MainWindow::rotateTissueLineDisp(double angle)
{
    p_CommonData->p_indicator->rotateAboutLocalAxisDeg(0,0,-1,angle);

    p_CommonData->indicatorRot = p_CommonData->indicatorRot + angle;
}

void MainWindow::on_pushButton_clicked()
{
    bool ok;
    QString TrialNoString = QInputDialog::getText(0, "Input Circle Radius",
                                         "Circle Rad:", QLineEdit::Normal, " ",
                                         &ok);
    p_CommonData->circRadius = TrialNoString.toDouble();
}

void MainWindow::rotateTissueLine(double angle)
{
    p_CommonData->p_tissueOne->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTwo->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueThree->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFour->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueFive->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueSix->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueSeven->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueEight->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueNine->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTen->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueEleven->rotateAboutLocalAxisDeg(0,0,-1,angle);
    p_CommonData->p_tissueTwelve->rotateAboutLocalAxisDeg(0,0,-1,angle);
}


void MainWindow::on_OneUp_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr1 = curr1 + 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}

void MainWindow::on_OneDown_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr1 = curr1 - 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}

void MainWindow::on_TwoUp_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr2 = curr2 + 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}

void MainWindow::on_TwoDown_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr2 = curr2 - 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}

void MainWindow::on_ThreeUp_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr3 = curr3 + 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}

void MainWindow::on_ThreeDown_clicked()
{
    double curr1 = p_CommonData->desJointInits[0];
    double curr2 = p_CommonData->desJointInits[1];
    double curr3 = p_CommonData->desJointInits[2];

    curr3 = curr3 - 1*PI/180;
    p_CommonData->desJointInits << curr1, curr2, curr3;
}
