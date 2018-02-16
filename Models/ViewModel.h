#ifndef VIEWMODEL_H
#define VIEWMODEL_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <vector>
#include "Types/datatypes.h"
#include "Classifiers/ClassifierLabels.h"
#include "Models/IViewModel.h"
#include "Descriptors/IPointDescriptor.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;

class ViewModel : public IViewModel
{
public:
    CloudType cloud;
    std::vector<IPointDescriptor*> descriptor;
};

#endif // VIEWMODEL_H
