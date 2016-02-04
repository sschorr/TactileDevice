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
#include <Qt/qfiledialog.h>
#include <Qt/qinputdialog.h>
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
};

#endif // MAINWINDOW_H
