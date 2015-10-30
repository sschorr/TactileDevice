#include <QApplication>
#include "mainwindow.h"
#include "cMotorController.h"

// Variable Declarations ============================================
cMotorController* First_Controller;


// Function Declarations ============================================
int Init_Motor_Controller();


// MAIN FUNCTION ====================================================
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    First_Controller = new cMotorController(0);
    First_Controller->open();
    delete First_Controller;
    
    return a.exec();
}



int Init_Motor_Controller(void)
{
    return 0;
}
