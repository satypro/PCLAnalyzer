/****************************************************************************
**
****************************************************************************/
#include <iostream>
#include <QtGui>
#include <QtOpenGL>
#include <math.h>
#include "Glwidget.h"

GLWidget::GLWidget(QWidget *parent, PCLAnalyzerWindow *mainWnd)
    : QGLWidget(parent)
{
     this->mainWnd = mainWnd;
     //QLayout * lt = this->layout();

    xShift = 0.0;
    yShift = 0.0;

    size_gl = 1.0;

    enableTranslation = 1;

    enableZoom = 1;

    enableZoomIn = 1;

    enableZoomOut = 1;

    enableRotation = 1;

    displaymode = 1;
    pointmode = 0;

    x_ang = 0.0;

    y_ang = 0.0;

      setFocusPolicy(Qt::ClickFocus);

   /* QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(advanceGears()));
    timer->start(20);*/
}

GLWidget::~GLWidget()
{
}

int GLWidget::w()
{
    return (this->width());

}

int GLWidget::h()
{
    return (this->height());
}

void GLWidget::render()
{
    mainWnd->display();
}

void GLWidget :: getVersions(void)
{
    const char* verstr = (const char*)glGetString(GL_VERSION) ;

    if (verstr)
            std::cout << "GL_VERSION : " << verstr << std :: endl ;
    else
            std::cout<< "Error in determining OpenGL version" << std :: endl ;

    return ;
}

void GLWidget :: setInitSize(void)
{
#ifdef QT_OPENGL_ES_1
    glOrthof(-2, 2, -2, 2, -100, 100) ;
#else
    glOrtho(-2, 2, -2, 2, -100, 100) ;
#endif

}
void GLWidget::initializeGL()
{
   /* static const GLfloat lightPos[4] = { 5.0f, 5.0f, 10.0f, 1.0f };
    static const GLfloat reflectance1[4] = { 0.8f, 0.1f, 0.0f, 1.0f };
    static const GLfloat reflectance2[4] = { 0.0f, 0.8f, 0.2f, 1.0f };
    static const GLfloat reflectance3[4] = { 0.2f, 0.2f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    gear1 = makeGear(reflectance1, 1.0, 4.0, 1.0, 0.7, 20);
    gear2 = makeGear(reflectance2, 0.5, 2.0, 2.0, 0.7, 10);
    gear3 = makeGear(reflectance3, 1.3, 2.0, 0.5, 0.7, 10);

    glEnable(GL_NORMALIZE);*/
   // glClearColor(1, 1, 1, 1) ;
    //glClearColor(0, 0, 0, 1) ;
    // glClearColor(0.4, 0, 0, 1) ;

     glClearColor(1, 1, 1, 1) ;
    glEnable( GL_LINE_SMOOTH );

    glPointSize(3.0f);
    // Depth Buffer Setup
    glClearDepth(1.0f);
    // Enables Depth Testing
    glEnable(GL_DEPTH_TEST);
    // The Type Of Depth Testing To Do
    glDepthFunc(GL_LEQUAL);
    // Really Nice Perspective Calculations
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    getVersions() ;

    glLoadIdentity( ) ;

    glViewport( 0, 0, w(), h() );
    // Orthographic projection
    glMatrixMode( GL_PROJECTION ) ;

    glLoadIdentity() ;

    setInitSize() ;

    glMatrixMode( GL_MODELVIEW ) ;

  //  glLoadIdentity() ;

}

void GLWidget::winClear()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

void GLWidget::paintGL()
{
   // if(pointmode != 10 && displaymode != 1 )
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
//  drawSpectrum();
    glPushMatrix();
    static float rotation_transform[4][4] ;
    // TODO Fix trackball.
    trackball.rotationMatrix( rotation_transform ) ;

    glScalef( float( size_gl ), float( size_gl ), float( size_gl ) );

    glTranslatef( xShift/size_gl, -yShift/size_gl, 0.5 );

    glMultMatrixf( &rotation_transform[0][0] ) ;

    render();

    glPopMatrix();
}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height) + 150;

   glViewport((width - side) / 2, (height - side) / 2, side, side);
  //   glViewport(0, 0, w(), h());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    setInitSize();
   // glFrustum(-1.0, +1.0, -1.0, 1.0, 5.0, 60.0);
    glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //glTranslated(0.0, 0.0, -40.0);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int xx = event->x() ;//- lastPos.x();
    int yy = event->y();// - lastPos.y();

    if (event->buttons() & Qt::LeftButton)
    {
        x_ang = lastPos.x();
        y_ang = lastPos.y();
        trackball.rotate((2.0 * x_ang - w()) / float(w()),(h() - 2.0 * y_ang) / float(h()),
                                    (2.0 * xx - w()) / float(w()), (h() - 2.0 * yy) / float(h()));
       // winClear();
        updateGL();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        oldPosX = lastPos.x();
        oldPosY = lastPos.y();
        newPosX = xx ;
        newPosY = yy ;
        // ortho_width and ortho_height come from
        // the orthographic projection dimensions.
        // If they should change, the developer
        // should update these two variables.
        const float ortho_width = 2 ;
        //const float ortho_height = 2 ;

        xShift += ortho_width*(newPosX - oldPosX)/double(w()) ;
        yShift += ortho_width*(newPosY - oldPosY)/double(h()) ;
        //winClear();
        updateGL();
        //setXRotation(xRot + 8 * yy);
        //setZRotation(zRot + 8 * xx);
    }

    oldPosX = xx ;
    oldPosY = yy ;
    lastPos = event->pos();
}

void GLWidget::mousWheelEvent(QWheelEvent *e)
{
    if(e->delta() > 0)
    {
        size_gl = (size_gl < 10.0) ? size_gl + 0.1 : 5. ;       //for zoom-in
    }
    else
    {
         size_gl = (size_gl > 0.6) ? size_gl - 0.1 : 0.6 ;          //for zoom-out
    }

   //winClear();
   updateGL();
}

void GLWidget::keyPressEvent(QKeyEvent *keyevent)
{
    //cout<<"detecting "<<endl;
     //((keyevent->key() == Qt::Key_Plus)  && (keyevent->modifiers().testFlag(Qt::ControlModifier)))
    switch (keyevent->key())
   {
        case Qt::Key_Equal:
            size_gl = (size_gl < 15.0) ? size_gl + 0.1 : 5. ;       //for zoom-in
            //winClear();
            updateGL();
        break;

        case Qt::Key_Minus:
            size_gl = (size_gl > 0.6) ? size_gl - 0.1 : 0.6 ;          //for zoom-out
            winClear();
            updateGL();
        break;

        case Qt::Key_D:
            displaymode = !(displaymode) ;
            //winClear();
            updateGL();
        break;

        case Qt::Key_G:
            pointmode = 0 ;
            //winClear();
            updateGL();
        break;

        case Qt::Key_H:
            pointmode = 1;
            //winClear();
            updateGL();
        break;

        case Qt::Key_M:
            pointmode = 2 ;
            winClear();
            updateGL();
        break;

        case Qt::Key_L:
            pointmode = 3 ;
           // winClear();
            updateGL();
        break;

        case Qt::Key_O:
            pointmode = 4 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_W:
            pointmode = 5 ;
           // winClear();
            updateGL();
        break;

        case Qt::Key_S:
            pointmode = 6 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_T:
            pointmode = 7 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_B:
            pointmode = 8 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_K:
            pointmode = 9 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_Z:
            pointmode = 10 ;
          //  winClear();
            updateGL();
        break;

        case Qt::Key_Escape:                      //for exit

              exit(0);
        break;
   }

}
