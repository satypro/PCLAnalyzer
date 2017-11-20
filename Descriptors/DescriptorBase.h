#ifndef DESCRIPTORBASE_H
#define DESCRIPTORBASE_H
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Types/datatypes.h"
#include "Config/Configuration.h"

class DescriptorBase
{
public:
    DescriptorBase(){ }
    virtual Eigen::Vector4f Get3DCentroid() = 0;
    virtual Eigen::Matrix3f ComputeCovarianceMatrix() = 0;
    virtual TensorType GetTensor() = 0;
    virtual TensorType Get3DVotingTensor() = 0;
    virtual Configuration* GetConfig() = 0;

    void virtual setSource(pcl::PointXYZ sourcePoint)
    {
        _sourcePoint = sourcePoint;
    }

    pcl::PointXYZ virtual getSource()
    {
        return _sourcePoint;
    }

    void virtual setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        _cloud = cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr virtual getCloud()
    {
        return _cloud;
    }
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::PointXYZ _sourcePoint;
};

#endif // DESCRIPTORBASE_H
