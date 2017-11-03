#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Types/datatypes.h"
#include "DescriptorBase.h"

class Descriptor : public DescriptorBase
{
public:
    Descriptor();
    Eigen::Vector4f Get3DCentroid();
    Eigen::Matrix3f ComputeCovarianceMatrix();
    TensorType GetTensor();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        DescriptorBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // DESCRIPTOR_H
