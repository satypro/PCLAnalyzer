#include "Descriptor.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

Descriptor::Descriptor()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* Descriptor::GetConfig()
{
    return _config;
}

Eigen::Vector4f Descriptor::Get3DCentroid()
{
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*getCloud(), xyz_centroid);

    return xyz_centroid;
}

Eigen::Matrix3f Descriptor::ComputeCovarianceMatrix()
{
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix(*getCloud(), Get3DCentroid(), covariance_matrix);

    return covariance_matrix;
}

TensorType Descriptor::GetTensor()
{
    TensorType tensor;

    Eigen::Matrix3f covariance_matrix = ComputeCovarianceMatrix();

    for (int k = 0 ; k < 3 ; k++)
    {
        for (int j = 0 ; j < 3; j++)
        {
            if (k == 0)
                tensor.evec0[j] = covariance_matrix(k, j);
            if (k == 1)
                tensor.evec1[j] = covariance_matrix(k, j);
            if (k == 2)
                tensor.evec2[j] = covariance_matrix(k, j);
        }
    }

    return tensor;
}
