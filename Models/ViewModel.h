#ifndef VIEWMODEL_H
#define VIEWMODEL_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <vector>
#include "Types/datatypes.h"
#include "Classifiers/ClassifierLabels.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;

enum Colors
{
    Red,
    Green,
    Blue
};

struct PointDescriptor
{
    probfeatnode featNode;
    featProps    featProp;
    glyphVars    glyph;
    ClassLabels  label;
};

struct ViewModel
{
    CloudType cloud;
    std::vector<PointDescriptor> descriptor;
};

#endif // VIEWMODEL_H
