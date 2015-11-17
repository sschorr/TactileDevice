#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "shared_data.h"
#include <QTimer>

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
    void on_verticalSliderX_valueChanged(int value);
    void on_verticalSliderY_valueChanged(int value);
    void on_verticalSliderZ_valueChanged(int value);
    void on_ZeroSliders_clicked();
};

#endif // MAINWINDOW_H
