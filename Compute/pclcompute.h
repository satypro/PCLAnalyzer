#ifndef PCLCOMPUTE_H
#define PCLCOMPUTE_H

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "datatypes.h"
#include "compute.h"

class PclCompute : public Compute
{
public:
    PclCompute();
    Eigen::Vector4f Get3DCentroid();
    Eigen::Matrix3f ComputeCovarianceMatrix(Eigen::Vector4f& centroid);
    tensorType GetTensor(Eigen::Matrix3f& covarianceMatrix);
    EigenVectorValue GetEigenResult(Eigen::Matrix3f& covarianceMatrix);
};

#endif // PCLCOMPUTE_H
