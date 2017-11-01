#ifndef RESULT_H
#define RESULT_H
#include <vector>
#include <pcl/common/eigen.h>
#include "Types/datatypes.h"

class Result
{
public:
    Eigen::Vector4f Centroid;
    Eigen::Matrix3f CovarianceMatrix;
    TensorType Tensor;
    Eigen::Matrix<float, 3 ,1> EigenValues;
    Eigen::Matrix<float, 3, 3> EigenVectors;
    Result();
};

#endif // RESULT_H
