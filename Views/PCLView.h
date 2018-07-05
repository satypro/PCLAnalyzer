#ifndef PCLVIEW_H
#define PCLVIEW_H
#include "IViews.h"
#include "Models/ViewModel.h"
#include <stdio.h>
#include <vector>
#include <string>
#include "Types/datatypes.h"
#include <pcl/io/pcd_io.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef K::Line_3                   Line3;
typedef K::Plane_3                  Plane;
typedef K::Point_3                  Point;
typedef K::Triangle_3               Triangle3;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;

class PCLView : public IViews
{
public:
    PCLView();
    void Show();
    void SetViewModel(IViewModel* model);
    void drawSpectrum(void);
    bool lasDisplay();
    bool plyDisplay();
    void writeTostl();
    bool renderStructFDs(int pointMode);
    void displayBoundary();
    void smoothTensorLines(std::string filename);
    void displayGraph();
    float scalarMin, scalarMax;

private:
    ViewModel* model;
    void renderCurvature();
    void idxPtCloudFeat();
    void csclcpDisplay();
    void sumeigen_Display();
    void planarityDisplay();
    void anisotropyDisplay();
    void sphericityDisplay();
    void donDisplay();
    void lineWireframe();
    void drawLineFatures();
    void contours();
    void eigenentropyDisplay();
    void omnivarianceDisplay();
    void linearityDisplay();
    void heightMap();
    void tensorLines();
    void triangulation_pointset();
    void labelsDisplay();

    CloudType _inCloud;
    std::vector <Line> _smoothTensorLines;
    std::vector <Line> _smoothTensorLinesCorrected;
    std::vector <float> errorTL;
    std::vector <float> errorCTL;
    std::vector <float> _intensity;
};

#endif // PCLVIEW_H
