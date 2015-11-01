#-------------------------------------------------
#
# Project created by QtCreator 2015-10-26T14:42:00
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TactileDevice
TEMPLATE = app


SOURCES += ./src/main.cpp\
           ./src/mainwindow.cpp\
           ./src/Win626.c \
           ./src/cMotorController.cpp \
           ./src/haptics_thread.cpp


HEADERS  += ./include/mainwindow.h\
            ./include/Win626.h\
            ./include/APP626.h \
            ./include/cMotorController.h \
            ./include/shared_data.h \
            ./include/haptics_thread.h \

FORMS    += ./mainwindow.ui

# Include path for qwt
INCLUDEPATH += "./include"
INCLUDEPATH += "./External/qwt-6.0.1/src"

# Include path for Chai3d and Openhaptics
INCLUDEPATH += "./External/chai3d-2.3.0/src"
INCLUDEPATH += "./External/chai3d-2.3.0/external/OpenGL/msvc"
INCLUDEPATH += "./External/chai3d-2.3.0/external/freeglut/include"
INCLUDEPATH += "./External/chai3d-2.3.0/external/BASS/include"
INCLUDEPATH += "./External/OpenHaptics_AE_3.1_BETA_3/include"
INCLUDEPATH += "./External/OpenHaptics_AE_3.1_BETA_3/utilities/include"

# Libraries for Chai3d and Openhaptics
LIBS += "./External/chai3d-2.3.0/lib/msvc10/debug/chai3d-debug.lib"
LIBS += "./External/chai3d-2.3.0/external/freeglut/lib/Win32/freeglut.lib"
LIBS += "./External/chai3d-2.3.0/external/BASS/lib/msvc/bass_msvc.lib"
LIBS += "./External/OpenHaptics_AE_3.1_BETA_3/lib/Win32/DebugAcademicEdition/hd.lib"
LIBS += "./External/OpenHaptics_AE_3.1_BETA_3/utilities/lib/Win32/Debug/hdu.lib"
LIBS += "./External/OpenHaptics_AE_3.1_BETA_3/utilities/lib/Win32/Debug/SnapConstraints.lib"
LIBS += "./External/OpenHaptics_AE_3.1_BETA_3/utilities/lib/Win32/Debug/glut32.lib"
