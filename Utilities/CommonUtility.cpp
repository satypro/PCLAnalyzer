#include "CommonUtility.h"
#include <iostream>

CommonUtility::CommonUtility()
{

}

EigenResult CommonUtility::ComputeEigen(Eigen::Matrix3f matrix)
{
    EigenResult result;

   /* Eigen:<Eigen::Matrix3f> eigensolver;
    eigensolver.compute(matrix);

    if (eigensolver.info() == Eigen::Success)
    {
        std::cout<<"Calculated Eigen vector and values "<<std::endl;
        result.EigenValues = eigensolver.eigenvalues();
        result.EigenVectors = eigensolver.eigenvectors();
    }
    */

    return result;
}

