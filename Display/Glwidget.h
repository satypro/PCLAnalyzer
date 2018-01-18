/****************************************************************************
**
****************************************************************************/
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include "pclanalyzerwindow.h"
#include "FastTrackball.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent, PCLAnalyzerWindow *mainWnd);
    ~GLWidget();

    void getVersions(void) ;

    void keyPressEvent(QKeyEvent *);

    void setDisplayMode(int displaymode)
    {
        this->displaymode = displaymode;
    }

    void setPointMode(int pointmode)
    {
        this->pointmode = pointmode;
    }

    void winClear();

public slots:

signals:

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mousWheelEvent(QWheelEvent *e);

private slots:

private:
    PCLAnalyzerWindow *mainWnd;

    double size_gl;

    Trackball trackball ;

    float xShift, yShift ;

    int oldPosX, oldPosY, newPosX, newPosY ;

    int enableTranslation, enableZoom, enableZoomIn, enableZoomOut, enableRotation;

    int displaymode, pointmode;

    GLdouble x_ang, y_ang ;

    int w();

    int h();

    QPoint lastPos;

    void setInitSize() ;

    void render();
};

#endif
