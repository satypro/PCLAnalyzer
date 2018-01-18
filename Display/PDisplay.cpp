#include "PDisplay.h"
#include <GL/gl.h>

PDisplay::PDisplay()
{

}

bool PDisplay::Print()
{
    glBegin(GL_POINTS);

    glColor3f(255, 0, 0);
    glVertex3f(1, 1, 2);
    glVertex3f(1, 1, 1);
    glVertex3f(1, 0.9, 2);
    glVertex3f(1, 0.8, 3);
    glVertex3f(1, 0.6, 4);
    glVertex3f(1, 0.5, 5);
    glVertex3f(1, 1, 2);

    glEnd();

    return true;
}
