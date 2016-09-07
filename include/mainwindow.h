#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "shared_data.h"
#include <QTimer>
#include <windows.h>
#include <iostream>
#include <sstream> // Required for stringstreams
#include <string.h>
#include <QString>
#include <ostream>
#include <istream>
#include <fstream>
#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <qfiledialog.h>
#include <qinputdialog.h>
#include <QKeyEvent>
#include <QThread>
#define SDL_MAIN_HANDLED
#include "SDL.h"
#include "OVRRenderContext.h"
#include "OVRDevice.h"
#include "Widget_OpenGLDisplay.h"
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>



using namespace chai3d;
using namespace std;


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    // functions
    void Initialize();
    void processEvents();

    // vars
    Widget_OpenGLDisplay* windowGLDisplay;
    shared_data* p_CommonData;

    double updateHz = 100;

    // duplicate data record vector to limit thread locking
    std::vector<DataRecordStruct> localDataRecorderVector;


    // qwtVars
    QwtPlotCurve *curve1;
    QwtPlotCurve *curve2;
    QwtPlotCurve *curve3;
    QwtPlotGrid *grid;
    QPolygonF points1;
    QPolygonF points2;
    QPolygonF points3;
    QPointF point;
    double i;

    Eigen::Vector3d localMotorAngles0;
    Eigen::Vector3d localJointAngles0;
    Eigen::Vector3d localCartesianPos0;
    Eigen::Vector3d localDesiredForce0;
    Eigen::Vector3d localDesiredMotorTorques0;
    Eigen::Vector3d localOutputVoltages0;
    Eigen::Vector3d localDesiredPos0;
    Eigen::Vector3d localDesiredJointAngle0;

    Eigen::Vector3d localMotorAngles1;
    Eigen::Vector3d localJointAngles1;
    Eigen::Vector3d localCartesianPos1;
    Eigen::Vector3d localDesiredForce1;
    Eigen::Vector3d localDesiredMotorTorques1;
    Eigen::Vector3d localOutputVoltages1;
    Eigen::Vector3d localDesiredPos1;
    Eigen::Vector3d localDesiredJointAngle1;

    // Controller slider params
    double KpScale;
    double KdScale;
    double KpInit;
    double KdInit;

    // Oculus Rift
    // display context
    chai3d::cOVRRenderContext renderContext;
    // oculus device
    chai3d::cOVRDevice oculusVR;

    //------------------------------------------------------------------------------
    // DECLARED MACROS
    //------------------------------------------------------------------------------

    // convert to resource path
    #define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


private:
    Ui::MainWindow *ui;

    // a timer for updating the gui
    QTimer GraphicsTimer;

    void keyPressEvent(QKeyEvent* a_event);

private slots:
    void UpdateGUIInfo();
    void on_CalibratePushButton_clicked();
    void on_ZeroSliders_clicked();
    void onGUIchanged();
    void on_startSin_clicked();
    void on_setDirectory_clicked();
    void on_turnOff_clicked();
    void on_palpationButton_clicked();
    void on_frictionButton_clicked();
    void on_startCircle_clicked();
    void on_loadProtocol_clicked();
    void on_startExperiment_clicked();
    void WriteDataToFile();
    void on_setNeutral_clicked();
    void on_setTrial_clicked();
    void on_dynamicEnvironment_clicked();
    void rotateTissueLineDisp(double angle);
    void rotateTissueLine(double angle);
    void on_startExperiment_3_clicked();
    void on_loadProtocol_3_clicked();
    void on_pushButton_clicked();
    void on_AllDown0_clicked();
    void on_AllDown1_clicked();
    void on_Mass_clicked();
    void on_Friction_clicked();
    void on_Size_clicked();
    void on_Stiffness_clicked();
    void on_loadProtocol_2_clicked();
    void on_startExperiment_2_clicked();
    bool CheckFingers();
    void on_impulseForward_clicked();
    void on_impulseBackward_clicked();
    void on_impulseRight_clicked();
    void on_impulseLeft_clicked();
    void on_impulseUp_clicked();
    void on_impulseDown_clicked();
    void on_scaleUp_clicked();
    void on_scaleDown_clicked();
    void on_impulseOff_clicked();
    void on_impulseTorquezNeg_clicked();
    void on_impulseTorquex_clicked();
    void on_impulseTorquexNeg_clicked();
    void on_impulseTorquey_clicked();
    void on_impulseTorqueyNeg_clicked();
    void on_impulseTorquez_clicked();
};

#endif // MAINWINDOW_H
