#ifndef VIEWMODEL_H
#define VIEWMODEL_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;

enum Colors
{
    Red,
    Green,
    Blue
};

struct ViewModel
{
    CloudType cloud;
    std::vector<Colors> PointColor;
};

#endif // VIEWMODEL_H
