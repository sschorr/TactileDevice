#-------------------------------------------------
#
# Project created by QtCreator 2015-10-26T14:42:00
#
#-------------------------------------------------

QT       += core gui opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TactileDevice
TEMPLATE = app

SOURCES += ./src/main.cpp\
           ./src/mainwindow.cpp\
           ./src/Win626.c \
           ./src/cMotorController.cpp \
           ./src/haptics_thread.cpp \
           ./src/c3DOFdevice.cpp \
           ./src/Widget_OpenGLDisplay.cpp \
           ./src/c3dofChaiDevice.cpp \
           ./src/trakSTAR.cpp \
           ./src/magtracker.cpp \
           ./src/experiment_thread.cpp \
           ./src/magTracker_thread.cpp \


HEADERS  += ./include/mainwindow.h\
            ./include/Win626.h\
            ./include/APP626.h \
            ./include/cMotorController.h \
            ./include/shared_data.h \
            ./include/haptics_thread.h \
            ./include/c3DOFdevice.h \
            ./include/Widget_OpenGLDisplay.h \
            ./include/c3dofChaiDevice.h \
            ./include/TrakSTAR.h \
            ./include/ATC3DG.h \
            ./include/stdafx.h \
            ./include/magtracker.h \
            ./include/SimpleIni.h \
            ./include/experiment_thread.h \
            ./include/magTracker_thread.h \

FORMS    += ./mainwindow.ui

#include path for Eigen
INCLUDEPATH += "./External"

# Include path for qwt
INCLUDEPATH += "./include"
INCLUDEPATH += "./External/qwt-6.0.1/src"

# Include path for Chai3d and Openhaptics
INCLUDEPATH += "./External/chai3d-3.0.0/src"
INCLUDEPATH += "./External/chai3d-3.0.0/external/glew/include"
INCLUDEPATH += "./External/chai3d-3.0.0/external/freeglut/include"
INCLUDEPATH += "./External/chai3d-3.0.0/external/BASS/include"
INCLUDEPATH += "./External/chai3d-3.0.0/external/Eigen"
INCLUDEPATH += "./External/chai3d-3.0.0/modules/ODE"
INCLUDEPATH += "./External/chai3d-3.0.0/modules/ODE/src"
INCLUDEPATH += "./External/chai3d-3.0.0/modules/ODE/external/ODE/include"
INCLUDEPATH += "./External/chai3d-3.0.0/modules/ODE/obj/Debug/Win32"

# Libraries for Chai3d and Openhaptics
LIBS += "./External/chai3d-3.0.0/lib/Debug/Win32/chai3d.lib"
#LIBS += "./External/chai3d-3.0.0/lib/Release/Win32/chai3d.lib"
LIBS += "./External/chai3d-3.0.0/extras/freeglut/lib/Debug/Win32/freeglut.lib"
LIBS += "./External/chai3d-3.0.0/modules/ODE/lib/Debug/Win32/CODE.lib"

# Library for Ascension trackewr
LIBS += "./lib/ATC3DG.lib"


#LIBS += "atls.lib"
#LIBS += "winmm.lib"
#LIBS += "odbc32.lib"
#LIBS += "odbccp32.lib"
#LIBS += "user32.lib"
#LIBS += "kernel32.lib"
#LIBS += "gdi32.lib"
#LIBS += "winspool.lib"
#LIBS += "shell32.lib"
#LIBS += "ole32.lib"
#LIBS += "oleaut32.lib"
#LIBS += "uuid.lib"
#LIBS += "comdlg32.lib"
#LIBS += "advapi32.lib"
#LIBS += "../External/qwt-6.0.1/lib/qwtd.lib"
