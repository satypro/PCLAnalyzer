#ifndef COMPUTE_H
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "datatypes.h"
#define COMPUTE_H


class Compute
{
public:
    virtual Eigen::Vector4f Get3DCentroid();
    virtual Eigen::Matrix3f ComputeCovarianceMatrix(Eigen::Vector4f& centroid);
    virtual tensorType GetTensor(Eigen::Matrix3f& covarianceMatrix);
    virtual EigenVectorValue GetEigenResult(Eigen::Matrix3f& covarianceMatrix);

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        _cloud = cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud()
    {
        return _cloud;
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};

#endif // COMPUTE_H
