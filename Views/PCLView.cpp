#include "PCLView.h"
#include <GL/gl.h>
#include "Descriptors/PointDescriptor.h"

PCLView::PCLView()
{
}

void PCLView::SetViewModel(IViewModel* model)
{
    this->model = static_cast<ViewModel*>(model);
}

void PCLView::Show()
{
    glBegin(GL_POINTS);
    /*for (size_t i = 0; i < this->model->cloud->points.size (); ++i)
    {
        if (isnan(this->model->cloud->points[i].x) || isnan(this->model->cloud->points[i].y) || isnan(this->model->cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }

        // To Get the original descriptor
        PointDescriptor* descriptor = static_cast<PointDescriptor*>(this->model->descriptor.at(i));

        glColor3f(0, 0, 0);

        if (descriptor->label == Point)
        {
            //glColor3f(255, 0, 0);
        }

        if (descriptor->label  == Curve)
        {
            glColor3f(0, 255, 0);
        }

        if (descriptor->label  == Disc)
        {
            glColor3f(0, 0, 255);
        }

        glVertex3f(this->model->cloud->points[i].x,
                   this->model->cloud->points[i].y,
                   this->model->cloud->points[i].z);
    }
    */

    for (size_t i = 0; i < this->model->cloud->points.size (); i++)
    {
        // To Get the original descriptor
        PointDescriptor* descriptor = static_cast<PointDescriptor*>(this->model->descriptor.at(i));

        if(descriptor->PtsProp[i] == 1)
        {
            glColor3f(0, 0, 1);
        }
        else if(descriptor->PtsProp[i] == 2)
        {
            glColor3f(1, 0, 0);
        }
        else  if(descriptor->PtsProp[i] == 3)
        {
            glColor3f(0, 1, 0);
        }

        glVertex3f(this->model->cloud->points[i].x,
                   this->model->cloud->points[i].y,
                   this->model->cloud->points[i].z);
    }
    glEnd();
}
