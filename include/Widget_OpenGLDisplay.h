#ifndef WIDGET_OPENGLDISPLAY_H
#define WIDGET_OPENGLDISPLAY_H

#include <QThread>
#include <time.h>
#include <chai3d.h>
#include <QGLWidget>
#include "Shared_Data.h"
#include "GL/glut.h"



class Widget_OpenGLDisplay : public QGLWidget
{
    Q_OBJECT
public:
    explicit Widget_OpenGLDisplay(QWidget *parent = 0);

    // pointer to the shared data
    shared_data *p_CommonData;

protected:
//    void processEvents();
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void timerEvent(QTimerEvent *event);
    
private:
    int m_displayWidth;
    int m_displayHeight;

signals:
    
public slots:
    
};

#endif // WIDGET_OPENGLDISPLAY_H
