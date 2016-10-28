#include "Widget_OpenGLDisplay.h"
#include "gl/GLU.h"

Widget_OpenGLDisplay::Widget_OpenGLDisplay(QWidget *parent) :
    QGLWidget(parent)
  , p_CommonData(NULL)
{
}

//===========================================================================
void Widget_OpenGLDisplay::initializeGL()
{
#ifndef OCULUS
    //initialization of OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    //glEnable(GLUT_RGB);

    startTimer(10);
#endif
}

void Widget_OpenGLDisplay::paintGL()
{
#ifndef OCULUS
    if(p_CommonData)
    {
        //render world
        p_CommonData->p_world->updateShadowMaps(false, false);
        p_CommonData->p_camera->renderView(m_displayWidth, m_displayHeight);

    }
#endif
}

void Widget_OpenGLDisplay::resizeGL(int width, int height)
{
#ifndef OCULUS
    //update the size of the viewport
    m_displayWidth = width;
    m_displayHeight = height;

    //proces resize keep good aspect ratio for 3D scene
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (GLfloat)width/(GLfloat)height, 0.01f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
#endif
}

//===========================================================================
void Widget_OpenGLDisplay::timerEvent(QTimerEvent *event)
{
#ifndef OCULUS
    updateGL();
#endif
}
