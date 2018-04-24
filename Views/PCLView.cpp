#include "PCLView.h"
#include <GL/gl.h>
#include "Descriptors/PointDescriptor.h"
#include "Color.h"
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

static int buildingNo = 1;
#define TENSORLINE_IMPLEMENTATION_TYPE 0

PCLView::PCLView()
{
}

void PCLView::SetViewModel(IViewModel* model)
{
    this->model = static_cast<ViewModel*>(model);
    _inCloud = this->model->cloud;
}

void PCLView::Show()
{
    glBegin(GL_POINTS);
    for (size_t i = 0; i < this->model->cloud->points.size (); ++i)
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

    /*
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
    }*/
    glEnd();

}

void PCLView::drawSpectrum(void)
{
    Color currentcolor ;
    glBegin(GL_QUAD_STRIP) ;

    for (int i = -3 ; i <= 4 ; i++)
    {
        double k = i + 3;
        currentcolor = currentcolor.findColor(k/7.0, 0.0, 1.0) ;
        glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
        glVertex3d(1.70, (double)i/4.0, 0.0) ;
        glVertex3d(1.80, (double)i/4.0, 0.0) ;
    }

    glEnd() ;
}

bool PCLView::lasDisplay()
{
    if(_inCloud->points.size() <= 0)
        return false;

    float minX = 100;
    float minY = 100;
    float minZ = 100;
    float maxX = -100;
    float maxY = -100;
    float maxZ = -100;

    // Intensity Map
    glBegin(GL_POINTS);

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
      double temp = _intensity[i];
      glColor3d(temp, temp, temp) ;
      glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
      if(_inCloud->points[i].x < minX)
          minX = _inCloud->points[i].x;
      if(_inCloud->points[i].y < minY)
          minY = _inCloud->points[i].y;
      if(_inCloud->points[i].z < minZ)
          minZ = _inCloud->points[i].z;
      if(_inCloud->points[i].x > maxX)
          maxX = _inCloud->points[i].x;
      if(_inCloud->points[i].y > maxY)
          maxY = _inCloud->points[i].y;
      if(_inCloud->points[i].z > maxZ)
          maxZ = _inCloud->points[i].z;
    }

    glEnd();

    return true;
}

bool PCLView :: plyDisplay()
{
    if(_inCloud->points.size() <= 0)
        return false;

    glBegin(GL_POINTS);

    glColor3f(0.5f, 0.5f, 0.5f);

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        glColor3f(0.5f, 0.5f, 0.5f);
        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }
    glEnd();

    return true;
}

void PCLView :: displayBoundary()
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(0,1,0) ;
    glBegin(GL_QUADS) ;
    glVertex3f(_inCloud->points[0].x,_inCloud->points[0].y,_inCloud->points[0].z);
    glVertex3f(_inCloud->points[3637].x,_inCloud->points[3637].y,_inCloud->points[3637].z);
    glVertex3f(_inCloud->points[3577].x,_inCloud->points[3577].y,_inCloud->points[3577].z);
    glVertex3f(_inCloud->points[1315].x,_inCloud->points[1315].y,_inCloud->points[1315].z);
    glEnd() ;

    glLineWidth(6.0f);
    glBegin(GL_LINES) ;
    glVertex3f(_inCloud->points[0].x,_inCloud->points[0].y,_inCloud->points[0].z);
    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);
    glVertex3f(_inCloud->points[1315].x,_inCloud->points[1315].y,_inCloud->points[1315].z);
    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);

    glVertex3f(_inCloud->points[3577].x,_inCloud->points[3577].y,_inCloud->points[3577].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);
    glVertex3f(_inCloud->points[3637].x,_inCloud->points[3637].y,_inCloud->points[3637].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);

    glVertex3f(_inCloud->points[9].x,_inCloud->points[9].y,_inCloud->points[9].z);
    glVertex3f(_inCloud->points[2642].x,_inCloud->points[2642].y,_inCloud->points[2642].z);
    glVertex3f(_inCloud->points[2642].x,_inCloud->points[2642].y,_inCloud->points[2642].z);
    glVertex3f(_inCloud->points[1218].x,_inCloud->points[1218].y,_inCloud->points[1218].z);
    glEnd() ;
}

void PCLView :: smoothTensorLines()
{
    std::ifstream tensorlinesFile;
    tensorlinesFile.open(filename.c_str());
    int numLines;
    int ndx,id;
    std::vector< std::vector<Point> > tls;
    std::vector<Point> temp;

    std::string word;

    while(tensorlinesFile >> word)
    {
        if(word.compare("POSITION:")==0)
        {
            tls.push_back(temp);
            temp.clear();
            tensorlinesFile >> word;
        }
        else
        {
            int p = atoi( word.c_str() );
            temp.push_back(Point(_inCloud->points[p].x,_inCloud->points[p].y,_inCloud->points[p].z));
        }
    }

    tls.push_back(temp);
    temp.clear();

    tensorlinesFile.close();

    std::cout << "Building no = " << buildingNo << endl;

    Point a0,a1,b0,b1,c0,c1,d0,d1,e0,e1,f0,f1,g0,g1;
    for(int i=0;i<tls.size();i++)
    {
        if(tls[i].size()<2)
        {
            continue;
        }
        else
        {
            Line3 line;
            CGAL::linear_least_squares_fitting_3(tls[i].begin(),tls[i].end(),line, CGAL::Dimension_tag<0>());

            float error = 0;
            for(int k=0;k<tls[i].size();k++)
            {
                error += squared_distance(tls[i][k],line);
            }
            errorTL.push_back(error);
              cout << error << endl;

            Point p = line.projection(tls[i][0]);
            Point q = line.projection(tls[i][1]);
              Point p = tls[i][0];
              Point q = tls[i][tls[i].size()-1];
              cout << p << " -------------- " << q << endl;
            Line temp;
            temp.p[0] = p.x(); temp.p[1] = p.y(); temp.p[2] = p.z();
            temp.q[0] = q.x(); temp.q[1] = q.y(); temp.q[2] = q.z();

            if(i==1) {
                a0 = Point(p);
                a1 = Point(q);
            }else if(i==2){
                b0 = Point(p);
                b1 = Point(q);
            }else if(i==3){
                c0 = Point(p);
                c1 = Point(q);
            }else if(i==4){
                d0 = Point(p);
                d1 = Point(q);
            }else if(i==5){
                e0 = Point(p);
                e1 = Point(q);
            }else if(i==6){
                f0 = Point(p);
                f1 = Point(q);
            }else if(i==7){
                g0 = Point(p);
                g1 = Point(q);
            }

            _smoothTensorLines.push_back(temp);
            }
    }

    buildingNo++;

    Point p1((c0.x()+e1.x()+d0.x())/3,(c0.y()+e1.y()+d0.y())/3,(c0.z()+e1.z()+d0.z())/3);
    Point p2((c1.x()+g1.x()+b0.x())/3,(c1.y()+g1.y()+b0.y())/3,(c1.z()+g1.z()+b0.z())/3);
    Point p3((f0.x()+e0.x())/2,(f0.y()+e0.y())/2,(f0.z()+e0.z())/2);
    Point p4((f1.x()+g0.x())/2,(f1.y()+g0.y())/2,(f1.z()+g0.z())/2);
    Point p5((d1.x()+a0.x())/2,(d1.y()+a0.y())/2,(d1.z()+a0.z())/2);
    Point p6((b1.x()+a1.x())/2,(b1.y()+a1.y())/2,(b1.z()+a1.z())/2);

    Line tempLine;
    tempLine.p[0] = p5.x(); tempLine.p[1] = p5.y(); tempLine.p[2] = p5.z();
    tempLine.q[0] = p6.x(); tempLine.q[1] = p6.y(); tempLine.q[2] = p6.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p6.x(); tempLine.p[1] = p6.y(); tempLine.p[2] = p6.z();
    tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p2.x(); tempLine.p[1] = p2.y(); tempLine.p[2] = p2.z();
    tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p1.x(); tempLine.p[1] = p1.y(); tempLine.p[2] = p1.z();
    tempLine.q[0] = p5.x(); tempLine.q[1] = p5.y(); tempLine.q[2] = p5.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
    tempLine.q[0] = p1.x(); tempLine.q[1] = p1.y(); tempLine.q[2] = p1.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p3.x(); tempLine.p[1] = p3.y(); tempLine.p[2] = p3.z();
    tempLine.q[0] = p4.x(); tempLine.q[1] = p4.y(); tempLine.q[2] = p4.z();
    _smoothTensorLinesCorrected.push_back(tempLine);
    tempLine.p[0] = p4.x(); tempLine.p[1] = p4.y(); tempLine.p[2] = p4.z();
    tempLine.q[0] = p2.x(); tempLine.q[1] = p2.y(); tempLine.q[2] = p2.z();
    _smoothTensorLinesCorrected.push_back(tempLine);

    for(int i=0;i<tls.size();i++) {
        if(tls[i].size()<2){continue;}
        else {
            Line3 line;
            if(i==1) {
                line = Line3(p5,p6);
            }else if(i==2){
                line = Line3(p6,p2);
            }else if(i==3){
                line = Line3(p2,p1);
            }else if(i==4){
                line = Line3(p1,p5);
            }else if(i==5){
                line = Line3(p3,p1);
            }else if(i==6){
                line = Line3(p3,p4);
            }else if(i==7){
                line = Line3(p4,p2);
            }
            float error = 0;
            for(int k=0;k<tls[i].size();k++) {
                error += squared_distance(tls[i][k],line);
            }
            cout << error << endl;
        }
    }

    //cout << "_smoothTensorLinesCorrected.size = " << _smoothTensorLinesCorrected.size() << endl;
}

void PCLView :: writeTostl()
{
    std::ofstream myfile;
        myfile.open ("triangulation.txt");
    int tNdx = 0;
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        for (size_t j = 0; j <this->model->descriptor.at(i)->featNode.triangles.size(); j++)
        {
            myfile << tNdx <<" " << this->model->descriptor.at(i)->featNode.triangles[j].pid
                   << " " << this->model->descriptor.at(i)->featNode.triangles[j].qid
                   << " " << this->model->descriptor.at(i)->featNode.triangles[j].rid
                   << std::endl;
            tNdx++;
        }
    }
    myfile.close();
}

void PCLView::idxPtCloudFeat()
{
    glBegin(GL_POINTS);

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        glColor3f(this->model->descriptor.at(i)->featNode.prob[1],
                this->model->descriptor.at(i)->featNode.prob[2],
                this->model->descriptor.at(i)->featNode.prob[0]);
        glVertex3f(_inCloud->points[i].x,
                   _inCloud->points[i].y,
                   _inCloud->points[i].z);
    }

    glEnd();
    return;
}

void PCLView::renderCurvature()
{
    Color currentcolor ;
    glBegin(GL_POINTS);

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
            currentcolor = currentcolor.findColor(model->descriptor[i]->featNode.featStrength[2], 0.0, 1.0) ;
            glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();
    return;
}

void PCLView :: csclcpDisplay()
{
    glPointSize(4.0);
    glBegin(GL_POINTS);

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float cl,cs,cp;
        cs = model->descriptor[i]->featNode.csclcp[2];
        cl = model->descriptor[i]->featNode.csclcp[1];
        cp = model->descriptor[i]->featNode.csclcp[0];
        glColor3f(cl, cs, cp);
        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();
    return;
}


void PCLView :: sumeigen_Display()
{
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(model->descriptor[i]->featNode.sum_eigen > max && model->descriptor[i]->featNode.sum_eigen!=-1)
            max = model->descriptor[i]->featNode.sum_eigen;
        if(model->descriptor[i]->featNode.sum_eigen < min && model->descriptor[i]->featNode.sum_eigen!=-1)
            min = model->descriptor[i]->featNode.sum_eigen;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (model->descriptor[i]->featNode.sum_eigen - min) / (max - min);
        if(model->descriptor[i]->featNode.sum_eigen==-1)
            glColor3f(0.0, 1.0, 0.0) ;
        else
            glColor3f(val, 0.0, val) ;
        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void PCLView :: planarityDisplay()
{
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(this->model->descriptor[i]->featNode.planarity > max)
            max = model->descriptor[i]->featNode.planarity;
        if(model->descriptor[i]->featNode.planarity < min)
            min = model->descriptor[i]->featNode.planarity;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = model->descriptor[i]->featNode.csclcp[1];
            if(val==-1)
                glColor3d(0.0, 1.0, 0.0) ;
            else
                glColor3d(val, 0.0, val) ;
        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void PCLView :: donDisplay()
{
    glPointSize(4.0);
    glBegin(GL_POINTS);

    float max = -1.0;
    float min = 10000.0;
    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(model->descriptor[i]->featNode.don > max)
            max = model->descriptor[i]->featNode.don;
        if(model->descriptor[i]->featNode.don < min)
            min = model->descriptor[i]->featNode.don;
    }

    for(size_t i = 0; i <_inCloud->points.size(); i++)
    {
        float val = (model->descriptor[i]->featNode.don - min)/(max - min);
        if(val==-1)
            glColor3f(0.0, 1.0, 0.0);
        else
            glColor3f(val, 0.0, val);

        if(val>=scalarMin && val<=scalarMax)
            glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }

    glEnd();

    return;
}

void PCLView :: anisotropyDisplay()
{
    glPointSize(6.0);
        glBegin(GL_POINTS);

        int count = 0;
        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = this->model->descriptor[i]->featNode.anisotropy;

            if(val == 0)
                count++;
            if(val==-1)
                glColor3f(1.0, 0.0, 0.0) ;
            else
                glColor3f(val, 0.0, val) ;
            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }

        glEnd();
        return;
}

void PCLView :: sphericityDisplay()
{
    glBegin(GL_POINTS);

        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = this->model->descriptor[i]->featNode.sphericity;
            if(val==-1)
                glColor3f(0.0, 1.0, 0.0) ;
            else
                glColor3f(0.0, val, val) ;

            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }

        glEnd();
        return;
}

void PCLView :: drawLineFatures()
{
    glBegin(GL_POINTS);

        for (size_t i = 0; i <_inCloud->points.size(); i++)
        {
            if(this->model->PtsProp[i] == 1)
            {
                glColor3f(0, 0, 1);
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
            else if(this->model->PtsProp[i] == 2)
            {
                glColor3f(1, 0, 0);
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
            else  if(this->model->PtsProp[i] == 3)
            {
                glColor3f(0, 1, 0);
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
        }

        glEnd();
        return;
}

void PCLView ::drawSpectrum(void)
{
    Color currentcolor ;
    glBegin(GL_QUAD_STRIP) ;

    for (int i = -3 ; i <= 4 ; i++)
    {
        double k = i + 3;
        currentcolor = currentcolor.findColor(k/7.0, 0.0, 1.0) ;
        glColor3d(currentcolor.red(), currentcolor.green(), currentcolor.blue()) ;
        glVertex3d(1.70, (double)i/4.0, 0.0) ;
        glVertex3d(1.80, (double)i/4.0, 0.0) ;
    }

    glEnd() ;
}

void PCLView :: lineWireframe()
{
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.5);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_TRIANGLES) ;
    glColor3f(1,0,0) ;

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(this->model->descriptor[i]->featNode.label==5){
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            if(_probval[_probval[i].triangles[j].pid].csclcp[1] > _probval[_probval[i].triangles[j].pid].csclcp[0] && _probval[_probval[i].triangles[j].pid].csclcp[1] > _probval[_probval[i].triangles[j].pid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].pid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].pid].csclcp[1], _probval[_probval[i].triangles[j].pid].csclcp[2], _probval[_probval[i].triangles[j].pid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].p[0],_probval[i].triangles[j].p[1],_probval[i].triangles[j].p[2]);

            if(_probval[_probval[i].triangles[j].qid].csclcp[1] > _probval[_probval[i].triangles[j].qid].csclcp[0] && _probval[_probval[i].triangles[j].qid].csclcp[1] > _probval[_probval[i].triangles[j].qid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].qid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].qid].csclcp[1], _probval[_probval[i].triangles[j].qid].csclcp[2], _probval[_probval[i].triangles[j].qid].csclcp[0]);

            glVertex3f(_probval[i].triangles[j].q[0],_probval[i].triangles[j].q[1],_probval[i].triangles[j].q[2]);

            if(_probval[_probval[i].triangles[j].rid].csclcp[1] > _probval[_probval[i].triangles[j].rid].csclcp[0] && _probval[_probval[i].triangles[j].rid].csclcp[1] > _probval[_probval[i].triangles[j].rid].csclcp[2])
                glColor3f(_probval[_probval[i].triangles[j].rid].csclcp[1],0,0);
            else
                glColor3f(_probval[_probval[i].triangles[j].rid].csclcp[1], _probval[_probval[i].triangles[j].rid].csclcp[2], _probval[_probval[i].triangles[j].rid].csclcp[0]);
            glVertex3f(_probval[i].triangles[j].r[0],_probval[i].triangles[j].r[1],_probval[i].triangles[j].r[2]);
        }
        }
    }

    glEnd() ;
}

void PCLView :: contours()
{
    glPointSize(1.0);
    glBegin(GL_POINTS);
    for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            double temp = _intensity[i];
            glColor3d(temp, temp, temp) ;
            if(this->model->descriptor[i].label == 5)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }
    glEnd();

    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.5);
    glBegin(GL_LINES) ;

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_LINES) ;
    glColor3f(1,0,0) ;

    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        for (size_t j = 0; j <_probval[i].triangles.size(); j++)
        {
            if(_probval[i].triangles[j].hasCL &&  (_probval[_probval[i].triangles[j].pid].label == 5 || _probval[_probval[i].triangles[j].qid].label == 5  || _probval[_probval[i].triangles[j].rid].label == 5) )
            {
                glVertex3f(_probval[i].triangles[j].CL_p1[0],_probval[i].triangles[j].CL_p1[1],_probval[i].triangles[j].CL_p1[2]);
                glVertex3f(_probval[i].triangles[j].CL_p2[0],_probval[i].triangles[j].CL_p2[1],_probval[i].triangles[j].CL_p2[2]);
            }
        }
    }

    glEnd() ;
    displayBoundary();
}

void PCLView :: eigenentropyDisplay()
{
    glPointSize(6.0);
        glBegin(GL_POINTS);

        float max = -1.0;
        float min = 10000.0;
        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            if(this->model->descriptor[i].eigenentropy > max)
                max =this->model->descriptor[i].eigenentropy;
            if(this->model->descriptor[i].eigenentropy < min)
                min = this->model->descriptor[i].eigenentropy;
        }

        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = (this->model->descriptor[i].eigenentropy - min) / (max - min);
            glColor3f(val, 0.0, val) ;

            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }

        glEnd();

        return;
}

void PCLView :: omnivarianceDisplay()
{
    glPointSize(6.0);
        glBegin(GL_POINTS);

        float max = -1.0;
        float min = 10000.0;
        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            if(this->model->descriptor[i]->featNode.omnivariance > max)
                max = this->model->descriptor[i]->featNode.omnivariance;
            if(this->model->descriptor[i]->featNode.omnivariance < min)
                min = this->model->descriptor[i]->featNode.omnivariance;
        }

        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = (this->model->descriptor[i]->featNode.omnivariance - min) / (max - min);
            glColor3f(val, 0.0, val) ;

            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }

        glEnd();

        return;
}

void PCLView :: linearityDisplay()
{
    // Line saliency Heat map
        glPointSize(6.0);
        glBegin(GL_POINTS);

        float max = -1.0;
        float min = 10000.0;
        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            if(this->model->descriptor[i]->featNode.csclcp[1] > max)
                max = this->model->descriptor[i]->featNode.csclcp[1];
            if(this->model->descriptor[i]->featNode.csclcp[1] < min)
                min = this->model->descriptor[i]->featNode.csclcp[1];
        }

        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = (this->model->descriptor[i]->featNode.csclcp[1] - min) / (max - min);
            glColor3f(val, 0.0, val) ;

            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
        }

        glEnd();

        return;
}

void PCLView :: heightMap()
{
    // Height Map
        float maxH = -1.0;
        float minH = 100.0;
        for(size_t i = 0; i <_inCloud->points.size(); i++)
        {
            if(_inCloud->points[i].z > maxH)
                maxH = _inCloud->points[i].z;
            if(_inCloud->points[i].z < minH)
                minH = _inCloud->points[i].z;
        }

        glPointSize(6.0);
        glBegin(GL_POINTS);
        for (size_t i = 0; i <_inCloud->points.size(); i++)
        {
            float val = (_inCloud->points[i].z - minH)/(maxH-minH);
            glColor3f(val,val,val) ;

            if(val>=scalarMin && val<=scalarMax)
                glVertex3f(_inCloud->points[i].x,_inCloud->points[i].y,_inCloud->points[i].z);
        }
        glEnd() ;
}

void PCLView :: tensorLines()
{
        // csclcpDisplay();

        if(TENSORLINE_IMPLEMENTATION_TYPE == 0) {
            // Streamline
            glPointSize(1.0);
            glBegin(GL_POINTS);
            for(size_t i = 0; i <_inCloud->points.size(); i++)
                {
                    double temp = _intensity[i];
                    glColor3d(temp, temp, temp) ;
                    if(this->model->descriptor[i]->featNode.label == 5)
                        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
                }
            glEnd();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor3f(1,0,0) ;

            glEnable(GL_LINE_SMOOTH);
            glLineWidth(1.5);
            glBegin(GL_LINES) ;

            for (size_t i = 0; i <_inCloud->points.size(); i++)
            {
                for (size_t j = 0; j <this->model->descriptor[i]->featNode.tensor_line.size(); j++)
                {
                    if(this->model->descriptor[this->model->descriptor[i]->featNode.tensor_line[j].ndxq]->featNode.label == 5)
                    {
                        glVertex3f(this->model->descriptor[i]->featNode.tensor_line[j].p[0],
                                this->model->descriptor[i]->featNode.tensor_line[j].p[1],
                                this->model->descriptor[i]->featNode.tensor_line[j].p[2]);

                        glVertex3f(this->model->descriptor[i]->featNode.tensor_line[j].q[0],
                                this->model->descriptor[i]->featNode.tensor_line[j].q[1],
                                this->model->descriptor[i]->featNode.tensor_line[j].q[2]);
                    }
                }
            }

            glEnd() ;
        }
        else if(TENSORLINE_IMPLEMENTATION_TYPE==1) {
            glPointSize(1.0);
            glBegin(GL_POINTS);
            for(size_t i = 0; i <_inCloud->points.size(); i++)
                {
                    double temp = _intensity[i];
                    glColor3d(temp, temp, temp) ;
                    if(this->model->descriptor[i].label == 5)
                        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
                }
            glEnd();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor3f(1,0,0) ;

            glEnable(GL_LINE_SMOOTH);
            glLineWidth(4);
            glBegin(GL_LINES) ;

            for (size_t i = 0; i <_smoothTensorLinesCorrected.size(); i++)
            {
                Line line = _smoothTensorLinesCorrected[i];
                glVertex3f(line.p[0],line.p[1],line.p[2]);
                glVertex3f(line.q[0],line.q[1],line.q[2]);

            }
            glEnd() ;
        }
}

void PCLView :: triangulation_pointset()
{
    glPointSize(6.0);
        glBegin(GL_POINTS);
        for(size_t i = 0; i <_inCloud->points.size(); i++)
            {
                double temp = _intensity[i];
                glColor3d(temp, temp, temp) ;
                glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
            }
        glEnd();

        glPointSize(6.0);
        glBegin(GL_POINTS) ;

        for (size_t i = 0; i <_inCloud->points.size(); i++)
        {
            for (size_t j = 0; j <this->model->descriptor[i]->featNode.triangles.size(); j++)
            {
                glColor3f(this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].pid]->featNode.csclcp[1],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].pid]->featNode.csclcp[2],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].pid]->featNode.csclcp[0]);

                glVertex3f(this->model->descriptor[i]->featNode.triangles[j].p[0],
                        this->model->descriptor[i]->featNode.triangles[j].p[1],
                        this->model->descriptor[i]->featNode.triangles[j].p[2]);

                glColor3f(this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].qid]->featNode.csclcp[1],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].qid]->featNode.csclcp[2],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].qid]->featNode.csclcp[0]);

                glVertex3f(this->model->descriptor[i]->featNode.triangles[j].q[0],
                        this->model->descriptor[i]->featNode.triangles[j].q[1],
                        this->model->descriptor[i]->featNode.triangles[j].q[2]);

                glColor3f(this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].rid]->featNode.csclcp[1],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].rid]->featNode.csclcp[2],
                        this->model->descriptor[this->model->descriptor[i]->featNode.triangles[j].rid]->featNode.csclcp[0]);

                glVertex3f(this->model->descriptor[i]->featNode.triangles[j].r[0],
                       this->model->descriptor[i]->featNode.triangles[j].r[1],
                       this->model->descriptor[i]->featNode.triangles[j].r[2]);
            }
        }
        glEnd() ;
}

void PCLView :: labelsDisplay()
{
    glBegin(GL_POINTS) ;
    for (size_t i = 0; i <_inCloud->points.size(); i++)
    {
        if(this->model->descriptor[i]->featNode.label==0)
            glColor3f(1,1,0.5) ;
        else if(this->model->descriptor[i]->featNode.label==1)
            glColor3f(0,1,1) ;
        else if(this->model->descriptor[i]->featNode.label==2)
            glColor3f(1,1,1) ;
        else if(this->model->descriptor[i]->featNode.label==3)
            glColor3f(1,1,0) ;
        else if(this->model->descriptor[i]->featNode.label==4)
            glColor3f(0,1,0.5) ;
        else if(this->model->descriptor[i]->featNode.label==5)
            glColor3f(0,0,1) ;
        else if(this->model->descriptor[i]->featNode.label==6)
            glColor3f(0,0.5,1) ;
        else if(this->model->descriptor[i]->featNode.label==7)
            glColor3f(0.5,1,0) ;
        else if(this->model->descriptor[i]->featNode.label==8)
            glColor3f(0,1,0) ;
        glVertex3f(_inCloud->points[i].x, _inCloud->points[i].y, _inCloud->points[i].z);
    }
    glEnd() ;
}
