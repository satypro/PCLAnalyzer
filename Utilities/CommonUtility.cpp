#include "CommonUtility.h"

CommonUtility::CommonUtility()
{

}

EigenResult CommonUtility::ComputeEigen(Eigen::Matrix3f matrix)
{
    EigenResult result;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matrix);

    if (eigensolver.info() == Eigen::Success)
    {
        result.EigenValues = eigensolver.eigenvalues();
        result.EigenVectors = eigensolver.eigenvectors();
    }

    return result;
}
