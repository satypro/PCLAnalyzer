#ifndef DATATYPES
#define DATATYPES
#include <pcl/common/eigen.h>

struct tensorType
{
    float evec0[3];
    float evec1[3];
    float evec2[3];
};

struct EigenVectorValue
{
    Eigen::Matrix<float, 3 ,1> EigenValues;
    Eigen::Matrix<float, 3, 3> EigenVectors;
};

#endif // DATATYPES

