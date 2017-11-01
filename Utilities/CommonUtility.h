#ifndef COMMONUTILITY_H
#define COMMONUTILITY_H
#include <pcl/common/eigen.h>
#include <Types/datatypes.h>

class CommonUtility
{
public:
    CommonUtility();
    static EigenResult ComputeEigen(Eigen::Matrix3f matrix);
};

#endif // COMMONUTILITY_H
