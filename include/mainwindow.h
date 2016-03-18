#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "shared_data.h"
#include <QTimer>
#include <windows.h>
#include <iostream>
#include <sstream> // Required for stringstreams
#include <string>
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

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void Initialize();
    shared_data* p_CommonData;
    Eigen::Vector3d localMotorAngles;
    Eigen::Vector3d localJointAngles;
    Eigen::Vector3d localCartesianPos;
    Eigen::Vector3d localDesiredForce;
    Eigen::Vector3d localDesiredMotorTorques;
    Eigen::Vector3d localOutputVoltages;
    Eigen::Vector3d localDesiredPos;
    Eigen::Vector3d localDesiredJointAngle;

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
    void on_stopRecord_clicked();
    void on_setDirectory_clicked();
    void on_turnOff_clicked();
    void on_palpationButton_clicked();
    void on_frictionButton_clicked();
    void on_humpButton_clicked();
    void on_hoopHumpButton_clicked();
    void on_startCircle_clicked();
    void on_loadProtocol_clicked();
    void on_startExperiment_clicked();
    void WriteDataToFile();
    void on_setNeutral_clicked();
    void on_setTrial_clicked();
    void on_dynamicEnvironment_clicked();
    void on_palpExp_clicked();
    void on_loadProtocol_2_clicked();
    void on_startExperiment_2_clicked();
    void on_paperEnvironment_clicked();
};

#endif // MAINWINDOW_H
