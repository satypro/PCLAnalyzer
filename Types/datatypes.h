#ifndef DATATYPES_H
#define DATATYPES_H
#include <pcl/common/eigen.h>

struct TensorType
{
    float evec0[3];
    float evec1[3];
    float evec2[3];
};

struct EigenResult
{
    Eigen::Matrix<float, 3 ,1> EigenValues;
    Eigen::Matrix<float, 3, 3> EigenVectors;
};

#endif // DATATYPES_H
