#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string.h>
#include "Widget_OpenGLDisplay.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
#ifndef OCULUS
    windowGLDisplay = new Widget_OpenGLDisplay(this->centralWidget());
    windowGLDisplay->setObjectName(QStringLiteral("windowGLDisplay"));
    windowGLDisplay->setGeometry(QRect(10, 20, 881, 561));
#endif
}

MainWindow::~MainWindow()
{    
    p_CommonData->hapticsThreadActive = false;
    delete ui;
}

void MainWindow::Initialize()
{
#ifdef OCULUS
    //--------------------------------------------------------------------------
    // SETUP OCULUS DISPLAY CONTEXT
    //--------------------------------------------------------------------------

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0)
    {
        cout << "failed initialization" << endl;
        QThread::msleep(1000);
    }

    if (!oculusVR.initVR())
    {
        cout << "failed to initialize Oculus" << endl;
        QThread::msleep(1000);
        SDL_Quit();
    }

    ovrSizei hmdResolution = oculusVR.getResolution();
    ovrSizei windowSize = { hmdResolution.w / 2, hmdResolution.h / 2 };

    renderContext.init("CHAI3D", 100, 100, windowSize.w, windowSize.h);
    SDL_ShowCursor(SDL_DISABLE);

    if (glewInit() != GLEW_OK)
    {
        oculusVR.destroyVR();
        renderContext.destroy();
        SDL_Quit();
    }

    if (!oculusVR.initVRBuffers(windowSize.w, windowSize.h))
    {
        oculusVR.destroyVR();
        renderContext.destroy();
        SDL_Quit();
    }
#endif

    // Set our current state
    p_CommonData->currentControlState = idleControl;

    // Initialize shared memory for OpenGL widget
    //ui->displayWidget->p_CommonData = p_CommonData;

#ifndef OCULUS
    windowGLDisplay->p_CommonData = p_CommonData;
#endif


    connect(this->ui->verticalSliderX0, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderY0, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderZ0, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderX1, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderY1, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->verticalSliderZ1, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->KpSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->KdSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->bandwidthAmpSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->bandwidthFreqSlider, SIGNAL(valueChanged(int)), this, SLOT(onGUIchanged()));

    connect(this->ui->AllDown0, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->AllDown1, SIGNAL(clicked()), this, SLOT(onGUIchanged()));

    connect(this->ui->normalBox, SIGNAL(stateChanged(int)), this, SLOT(onGUIchanged()));
    connect(this->ui->lateralBox, SIGNAL(stateChanged(int)), this, SLOT(onGUIchanged()));

    connect(this->ui->sliderControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));
    connect(this->ui->VRControl, SIGNAL(clicked()), this, SLOT(onGUIchanged()));

    // init slider stuff
    this->KpScale = 40.0; this->KpInit = 50.0;
    this->KdScale = 0.24; this->KdInit = 50.0;
    this->ui->KpSlider->setValue(KpInit);
    this->ui->KdSlider->setValue(KdInit);

    this->ui->bandwidthAmpSlider->setValue(70);
    this->ui->bandwidthFreqSlider->setValue(10);

    p_CommonData->indicatorRot = 0;
    p_CommonData->tissueRot = 0;

    p_CommonData->desJointInits0 = p_CommonData->wearableDelta0->GetJointAngles();
    p_CommonData->desJointInits1 = p_CommonData->wearableDelta1->GetJointAngles();

    ui->normalBox->setChecked(true);
    ui->lateralBox->setChecked(true);
    p_CommonData->flagNormal = true;
    p_CommonData->flagLateral = true;
    p_CommonData->device0Initing = false;
    p_CommonData->device1Initing = false;

#ifdef QWT
    ///////////////
    // QWT INITS //
    ///////////////
    i = 0;

    ui->qwtPlot->setTitle("Test Plot");
    ui->qwtPlot->setCanvasBackground(Qt::white);
    ui->qwtPlot->setAxisScale(QwtPlot::xBottom, 0, 10.0);
    ui->qwtPlot->setAxisScale(QwtPlot::yLeft, 8.0, 18.0);
    ui->qwtPlot->updateAxes();

    ui->qwtPlot->show();
    ui->qwtPlot->setAutoReplot(true);

#endif

    GraphicsTimer.setInterval(1/updateHz*1000);
    GraphicsTimer.start();
    connect(&GraphicsTimer, SIGNAL(timeout()), this, SLOT(UpdateGUIInfo()));
    UpdateGUIInfo();

}


void MainWindow::UpdateGUIInfo()
{

#ifdef OCULUS
    // Handle Oculus events
    // handle key presses
    processEvents();

    // start rendering
    oculusVR.onRenderStart();

    // render frame for each eye
    for (int eyeIndex = 0; eyeIndex < ovrEye_Count; eyeIndex++)
    {
        // retrieve projection and modelview matrix from oculus
        cTransform projectionMatrix, modelViewMatrix;
        oculusVR.onEyeRender(eyeIndex, projectionMatrix, modelViewMatrix);

        p_CommonData->p_camera->m_useCustomProjectionMatrix = true;
        p_CommonData->p_camera->m_projectionMatrix = projectionMatrix;

        p_CommonData->p_camera->m_useCustomModelViewMatrix = true;
        p_CommonData->p_camera->m_modelViewMatrix = modelViewMatrix;

        // render world
        ovrSizei size = oculusVR.getEyeTextureSize(eyeIndex);
        p_CommonData->p_camera->renderView(size.w, size.h, 0, C_STEREO_LEFT_EYE, false);

        // finalize rendering
        oculusVR.onEyeRenderFinish(eyeIndex);
    }

    // update frames
    oculusVR.submitFrame();
    oculusVR.blitMirror();
    SDL_GL_SwapWindow(renderContext.window);
#endif

    localMotorAngles0 = p_CommonData->wearableDelta0->GetMotorAngles();
    localJointAngles0 = p_CommonData->wearableDelta0->GetJointAngles();
    localCartesianPos0 = p_CommonData->wearableDelta0->GetCartesianPos();
    localDesiredForce0 = p_CommonData->wearableDelta0->ReadDesiredForce();
    localOutputVoltages0 = p_CommonData->wearableDelta0->ReadVoltageOutput();
    localDesiredPos0 = p_CommonData->wearableDelta0->ReadDesiredPos();
    localDesiredJointAngle0 = p_CommonData->desJointInits0;//p_CommonData->wearableDelta0->CalcInverseKinJoint();

    localMotorAngles1 = p_CommonData->wearableDelta1->GetMotorAngles();
    localJointAngles1 = p_CommonData->wearableDelta1->GetJointAngles();
    localCartesianPos1 = p_CommonData->wearableDelta1->GetCartesianPos();
    localDesiredForce1 = p_CommonData->wearableDelta1->ReadDesiredForce();
    localOutputVoltages1 = p_CommonData->wearableDelta1->ReadVoltageOutput();
    localDesiredPos1 = p_CommonData->wearableDelta1->ReadDesiredPos();
    localDesiredJointAngle1 = p_CommonData->desJointInits1;//p_CommonData->wearableDelta0->CalcInverseKinJoint();

#ifdef QWT
    ////////////////////////////////////////////////////
    // update real time plotting
    ////////////////////////////////////////////////////
    ui->qwtPlot->setAxisAutoScale(QwtPlot::yLeft);
    i += 1/updateHz;

    // assign values to first curve
    points1 << QPointF( i , p_CommonData->wearableDelta0->KdEffort );
    if (points1.length() > 5*updateHz)
        points1.removeFirst();

    // assign values to second curve
    points2 << QPointF( i , p_CommonData->wearableDelta0->KpEffort );
    if (points2.length() > 5*updateHz)
        points2.removeFirst();

    // assign values to third curve
    points3 << QPointF( i , localCartesianPos0[2] );
    if (points3.length() > 5*updateHz)
        points3.removeFirst();

    QRectF rect = points1.boundingRect();
    ui->qwtPlot->setAxisScale(QwtPlot::xBottom, rect.left(), rect.right());

    ui->qwtPlot->detachItems(QwtPlotItem::Rtti_PlotItem, true);
//    curve1 = new QwtPlotCurve("Points1");
//    curve1->setSamples( points1 );
//    curve1->setPen(* new QPen(Qt::blue));
//    curve1->attach(ui->qwtPlot);

//    curve2 = new QwtPlotCurve("Points2");
//    curve2->setSamples( points2 );
//    curve2->setPen(* new QPen(Qt::red));
//    curve2->attach(ui->qwtPlot);

    curve3 = new QwtPlotCurve("Points3");
    curve3->setSamples( points3 );
    curve3->setPen(* new QPen(Qt::black));
    curve3->attach(ui->qwtPlot);
#endif

    // index device
    ui->MotorAngleLCDNumber1_0->display(localMotorAngles0[0]*180/PI);
    ui->MotorAngleLCDNumber2_0->display(localMotorAngles0[1]*180/PI);
    ui->MotorAngleLCDNumber3_0->display(localMotorAngles0[2]*180/PI);
    ui->JointAngleLCDNumber1_0->display(localJointAngles0[0]*180/PI);
    ui->JointAngleLCDNumber2_0->display(localJointAngles0[1]*180/PI);
    ui->JointAngleLCDNumber3_0->display(localJointAngles0[2]*180/PI);
    ui->CartesianXLCDNumber0->display(localCartesianPos0[0]);
    ui->CartesianYLCDNumber0->display(localCartesianPos0[1]);
    ui->CartesianZLCDNumber0->display(localCartesianPos0[2]);
    ui->desAngle1_0->display(localDesiredJointAngle0[0]*180/PI);
    ui->desAngle2_0->display(localDesiredJointAngle0[1]*180/PI);
    ui->desAngle3_0->display(localDesiredJointAngle0[2]*180/PI);
    ui->DesiredForceX0->display(localDesiredForce0[0]);
    ui->DesiredForceY0->display(localDesiredForce0[1]);
    ui->DesiredForceZ0->display(localDesiredForce0[2]);
    ui->VoltageLCD1_0->display((double)(localOutputVoltages0[0]));
    ui->VoltageLCD2_0->display((double)(localOutputVoltages0[1]));
    ui->VoltageLCD3_0->display((double)(localOutputVoltages0[2]));
    ui->DesX0->display(localDesiredPos0[0]);
    ui->DesY0->display(localDesiredPos0[1]);
    ui->DesZ0->display(localDesiredPos0[2]);

    // thumb device
    ui->MotorAngleLCDNumber1_1->display(localMotorAngles1[0]*180/PI);
    ui->MotorAngleLCDNumber2_1->display(localMotorAngles1[1]*180/PI);
    ui->MotorAngleLCDNumber3_1->display(localMotorAngles1[2]*180/PI);
    ui->JointAngleLCDNumber1_1->display(localJointAngles1[0]*180/PI);
    ui->JointAngleLCDNumber2_1->display(localJointAngles1[1]*180/PI);
    ui->JointAngleLCDNumber3_1->display(localJointAngles1[2]*180/PI);
    ui->CartesianXLCDNumber1->display(localCartesianPos1[0]);
    ui->CartesianYLCDNumber1->display(localCartesianPos1[1]);
    ui->CartesianZLCDNumber1->display(localCartesianPos1[2]);
    ui->desAngle1_1->display(localDesiredJointAngle1[0]*180/PI);
    ui->desAngle2_1->display(localDesiredJointAngle1[1]*180/PI);
    ui->desAngle3_1->display(localDesiredJointAngle1[2]*180/PI);
    ui->DesiredForceX1->display(localDesiredForce1[0]);
    ui->DesiredForceY1->display(localDesiredForce1[1]);
    ui->DesiredForceZ1->display(localDesiredForce1[2]);
    ui->VoltageLCD1_1->display((double)(localOutputVoltages1[0]));
    ui->VoltageLCD2_1->display((double)(localOutputVoltages1[1]));
    ui->VoltageLCD3_1->display((double)(localOutputVoltages1[2]));
    ui->DesX1->display(localDesiredPos1[0]);
    ui->DesY1->display(localDesiredPos1[1]);
    ui->DesZ1->display(localDesiredPos1[2]);

    ui->lcdNumberHapticRate->display(p_CommonData->hapticRateEstimate);
    ui->lcdBandAmp->display(p_CommonData->bandSinAmpDisp);
    ui->lcdBandFreq->display(p_CommonData->bandSinFreqDisp);
    ui->lcdKp->display(p_CommonData->jointKp);
    ui->lcdKd->display(p_CommonData->jointKd);
    ui->trialNo->display(p_CommonData->trialNo);
    ui->pairNo->display(p_CommonData->pairNo);

    //calibrate if startup process over
    if(p_CommonData->calibClock.timeoutOccurred())
    {
        p_CommonData->calibClock.stop();
        p_CommonData->calibClock.reset();

        on_CalibratePushButton_clicked();
    }

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

    case endExperiment:
        ui->directions->setText("Experiment over, please contact administrator");
        break;

    case palpationLineBreak:
        ui->directions->setText("Please take a break.  Press 'R' to continue");
        break;

    case trialBreak:
        ui->directions->setText("Please take a break. \nPress 'Q' to continue.");
        ui->selection->setText("Higher Friction:");
        break;
    }
}
void MainWindow::onGUIchanged()
{

    if(ui->lateralBox->isChecked())
    {
        p_CommonData->flagLateral = true;
    }
    else
    {
        p_CommonData->flagLateral = false;
    }

    if(ui->normalBox->isChecked())
    {
        p_CommonData->flagNormal = true;
    }
    else
    {
        p_CommonData->flagNormal = false;
    }

    if(ui->initJoints->isChecked())
    {
        p_CommonData->currentControlState = initCalibControl;
    }
    if(ui->sliderControl->isChecked())
    {
        p_CommonData->currentControlState = sliderControlMode;

        double xSlider0 = this->ui->verticalSliderX0->value()/22.2;
        double ySlider0 = this->ui->verticalSliderY0->value()/22.2;
        double zSlider0 = this->ui->verticalSliderZ0->value()/22.2+p_CommonData->wearableDelta0->neutralPos[2];
        Eigen::Vector3d tempDesiredPos0(xSlider0, ySlider0, zSlider0);
        p_CommonData->wearableDelta0->SetDesiredPos(tempDesiredPos0);

        double xSlider1 = this->ui->verticalSliderX1->value()/22.2;
        double ySlider1 = this->ui->verticalSliderY1->value()/22.2;
        double zSlider1 = this->ui->verticalSliderZ1->value()/22.2+p_CommonData->wearableDelta1->neutralPos[2];
        Eigen::Vector3d tempDesiredPos1(xSlider1, ySlider1, zSlider1);
        p_CommonData->wearableDelta1->SetDesiredPos(tempDesiredPos1);
    }

    else if(ui->VRControl->isChecked())
    {
        //let haptics thread determine desired position
        p_CommonData->currentControlState = VRControlMode;
    }

    double KpSlider = this->ui->KpSlider->value()*KpScale;
    double KdSlider = this->ui->KdSlider->value()*KdScale;

    double bandwidthAmp = this->ui->bandwidthAmpSlider->value()/20.0;
    double bandwidthFreq = this->ui->bandwidthFreqSlider->value()/3;

    p_CommonData->jointKp = KpSlider;
    p_CommonData->jointKd = KdSlider;
    p_CommonData->bandSinAmpDisp = bandwidthAmp;
    p_CommonData->bandSinFreqDisp = bandwidthFreq;

    ui->directions->setText("No experiment currently running");

    UpdateGUIInfo();
}


void MainWindow::on_CalibratePushButton_clicked()
{
    p_CommonData->wearableDelta0->ZeroEncoders();
    p_CommonData->desJointInits0 = p_CommonData->wearableDelta0->GetJointAngles();

    p_CommonData->wearableDelta1->ZeroEncoders();
    p_CommonData->desJointInits1 = p_CommonData->wearableDelta1->GetJointAngles();

    onGUIchanged();
}

void MainWindow::on_ZeroSliders_clicked()
{
    ui->verticalSliderX0->blockSignals(true);
    ui->verticalSliderY0->blockSignals(true);
    ui->verticalSliderZ0->blockSignals(true);
    ui->verticalSliderX1->blockSignals(true);
    ui->verticalSliderY1->blockSignals(true);
    ui->verticalSliderZ1->blockSignals(true);
    ui->verticalSliderX0->setValue(0);
    ui->verticalSliderY0->setValue(0);
    ui->verticalSliderZ0->setValue(0);
    ui->verticalSliderX1->setValue(0);
    ui->verticalSliderY1->setValue(0);
    ui->verticalSliderZ1->setValue(0);
    ui->verticalSliderX0->blockSignals(false);
    ui->verticalSliderY0->blockSignals(false);
    ui->verticalSliderZ0->blockSignals(false);
    ui->verticalSliderX1->blockSignals(false);
    ui->verticalSliderY1->blockSignals(false);
    ui->verticalSliderZ1->blockSignals(false);
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
            if(p_CommonData->subjectAnswer == 1 || p_CommonData->subjectAnswer == 2)
            {
                if(p_CommonData->currentExperimentState == frictionTrial)
                {
                    if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
                    {

                        WriteDataToFile();

                        // check if next trial is a break
                        QString nextTrialType = p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo + 1)).toStdString().c_str(), "type", NULL /*default*/);
                        if (nextTrialType == "break")
                        {
                            p_CommonData->currentExperimentState = trialBreak;
                            p_CommonData->recordFlag = false;
                            p_CommonData->debugData.clear();
                            p_CommonData->trialNo = p_CommonData->trialNo + 1;
                        }

                        else
                        {
                            p_CommonData->pairNo = 1;
                            p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
                            p_CommonData->subjectAnswer = 0;
                            ui->selection->setText("Higher Friction:");
                            double max = 0.01; double min = -0.01;
                            double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                            p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
                            p_CommonData->trialNo = p_CommonData->trialNo + 1;
                            p_CommonData->recordFlag = true;
                        }
                    }
                }
            }
        }


        // palpation Line experiment
        if(p_CommonData->currentExperimentState == palpationLineTrial)
        {
            // check that we are not currently embedded in tissue
            if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
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
            if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
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
            if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
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
        if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
        {
            if(p_CommonData->pairNo == 2)
            {
                p_CommonData->pairNo = 1;
                p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
                double max = 0.01; double min = -0.01;
                double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
            }
        }
    }

    if (a_event->key() == Qt::Key_1)
    {
        if(p_CommonData->currentExperimentState == frictionTrial)
        {
            if(p_CommonData->pairNo == 2)
            {
                p_CommonData->subjectAnswer = 1;
                ui->selection->setText("Higher Friction: 1 (blue)");
            }
        }
    }

    if (a_event->key() == Qt::Key_2)
    {
        if(p_CommonData->currentExperimentState == frictionTrial)
        {
            if(p_CommonData->pairNo == 2)
            {
                p_CommonData->subjectAnswer = 2;
                ui->selection->setText("Higher Friction: 2 (red)");
            }
        }
    }

    if (a_event->key() == Qt::Key_Q)
    {
        if(p_CommonData->currentExperimentState == trialBreak)
        {
            // check if next trial is end
            QString nextTrialType = p_CommonData->frictionProtocolFile.GetValue((QString("trial ") + QString::number(p_CommonData->trialNo + 1)).toStdString().c_str(), "type", NULL /*default*/);
            if (nextTrialType == "end")
            {
                p_CommonData->currentExperimentState = endExperiment;
                ui->directions->setText("Experiment Completed, please contact administrator");
            }
            else
            {
                p_CommonData->currentExperimentState = frictionTrial;
                p_CommonData->pairNo = 1;
                p_CommonData->p_expFrictionBox->m_material->setBlueAqua();
                p_CommonData->subjectAnswer = 0;
                ui->selection->setText("Higher Friction:");
                double max = 0.01; double min = -0.01;
                double randPos = ((double) rand()*(max-min)/(double)RAND_MAX+min);
                p_CommonData->p_expFrictionBox->setLocalPos(0,0,randPos);
                p_CommonData->trialNo = p_CommonData->trialNo + 1;
                p_CommonData->recordFlag = true;
            }
        }

        else if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
        {
            if(p_CommonData->pairNo == 1)
            {
                p_CommonData->pairNo = 2;
                p_CommonData->p_expFrictionBox->m_material->setRedCrimson();
                double max = 0.01; double min = -0.01;
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

void MainWindow::on_dynamicEnvironment_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentEnvironmentState = dynamicBodies;
}

void MainWindow::on_loadProtocol_clicked()
{
    //Open dialog box to get protocol file and save into variable
    QString temp = QFileDialog::getOpenFileName();
    p_CommonData->frictionProtocolLocation = temp;
    int error = p_CommonData->frictionProtocolFile.LoadFile(temp.toStdString().c_str());
    qDebug() << p_CommonData->frictionProtocolLocation;
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
    p_CommonData->debugData.clear();
}

void MainWindow::on_startExperiment_3_clicked()
{
    p_CommonData->environmentChange = true;
    p_CommonData->currentExperimentState = palpationLineTrial;
    p_CommonData->currentEnvironmentState = experimentPalpationLine;
    p_CommonData->currentControlState = VRControlMode;
    ui->VRControl->setChecked(true);

    // check that we are not currently embedded in tissue
    if(!(localDesiredPos0[2] < p_CommonData->wearableDelta0->neutralPos[2]))
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
    p_CommonData->wearableDelta0->neutralPos[2] = localCartesianPos0[2];
    p_CommonData->wearableDelta1->neutralPos[2] = localCartesianPos1[2];
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

void MainWindow::processEvents()
{
#ifdef OCULUS
    SDL_Event event;

    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_KEYDOWN:
            // esc
            if (event.key.keysym.sym == SDLK_q || event.key.keysym.sym == SDLK_ESCAPE)
            {
                close();
                break;
            }

            // spacebar
            if (event.key.keysym.sym == SDLK_SPACE)
            {
                oculusVR.recenterPose();
                break;
            }

            break;

        case SDL_QUIT:
            close();
            break;

        default:
            break;
        }
    }
#endif
}


void MainWindow::on_AllDown0_clicked()
{
    p_CommonData->calibClock.reset();
    p_CommonData->calibClock.setTimeoutPeriodSeconds(5.0);
    p_CommonData->calibClock.start();
    p_CommonData->device0Initing = true;
}

void MainWindow::on_AllDown1_clicked()
{
    p_CommonData->calibClock.reset();
    p_CommonData->calibClock.setTimeoutPeriodSeconds(5.0);
    p_CommonData->calibClock.start();
    p_CommonData->device1Initing = true;
}
