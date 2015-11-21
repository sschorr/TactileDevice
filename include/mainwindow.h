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



private slots:
    void UpdateGUIInfo();
    void on_CalibratePushButton_clicked();
    void on_ZeroSliders_clicked();
    void on_GUI_changed();
};

#endif // MAINWINDOW_H
