#include "PCLView.h"
#include <GL/gl.h>

PCLView::PCLView()
{

}

void PCLView::Show()
{
    glBegin(GL_POINTS);
    for (size_t i = 0; i < model.cloud->points.size (); ++i)
    {
        if (isnan(model.cloud->points[i].x) || isnan(model.cloud->points[i].y) || isnan(model.cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }
        glColor3f(0, 0, 0);

        if (model.descriptor.at(i).label == Point)
        {
            //glColor3f(255, 0, 0);
        }

        if (model.descriptor.at(i).label  == Curve)
        {
            glColor3f(0, 255, 0);
        }

        if (model.descriptor.at(i).label  == Disc)
        {
            glColor3f(0, 0, 255);
        }

        glVertex3f(model.cloud->points[i].x, model.cloud->points[i].y, model.cloud->points[i].z);
    }
    glEnd();
}
