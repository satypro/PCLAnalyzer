#include "Descriptor.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <stdlib.h>

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

TensorType Descriptor::Get3DVotingTensor()
{
    TensorType tensor;
    TensorType result;
    float weight;
    Eigen::Matrix<double, 3, 1> V;
    char* pEnd;
    float lambdaN = ::strtof(_config->GetValue("lambdaN").c_str(), &pEnd);

    for(int j =0; j < 3; j++)
    {
        result.evec0[j] = 0;
        result.evec1[j] = 0;
        result.evec2[j] = 0;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = getCloud();

    for (size_t i = 0; i < neighbourCloud->points.size() ; i++)
    {
        if (MakeVector(getSource(), neighbourCloud->points[i], &V))
        {
            tensor = Compute3DBallVote(V, &weight);

            std::cout<<"Vector :"<<V(0,0)<<" " << V(1, 0) <<" "<<V(2,0)<<std::endl;
            std::cout<<"Tesnsor Row 0 :"<<tensor.evec0[0] <<" "<<tensor.evec0[1]<<" "<< tensor.evec0[2] << std::endl;
            std::cout<<"Tesnsor Row 1 :"<<tensor.evec1[0] <<" "<<tensor.evec1[1]<<" "<< tensor.evec1[2] << std::endl;
            std::cout<<"Tesnsor Row 2 :"<<tensor.evec2[0] <<" "<<tensor.evec2[1]<<" "<< tensor.evec2[2] << std::endl;

            for(int j =0; j < 3; j++)
            {
                result.evec0[j] = result.evec0[j] + lambdaN * tensor.evec0[j];
                result.evec1[j] = result.evec1[j] + lambdaN * tensor.evec1[j];
                result.evec2[j] = result.evec2[j] + lambdaN * tensor.evec2[j];
            }
        }
    }

    std::cout<<"Tesnsor  R Row 0 :"<<result.evec0[0] <<" "<<result.evec0[1]<<" "<< result.evec0[2] << std::endl;
    std::cout<<"Tesnsor  R Row 1 :"<<result.evec1[0] <<" "<<result.evec1[1]<<" "<< result.evec1[2] << std::endl;
    std::cout<<"Tesnsor  R Row 2 :"<<result.evec2[0] <<" "<<result.evec2[1]<<" "<< result.evec2[2] << std::endl;

    return result;
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

/*Private methods*/
bool Descriptor::MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V)
{
    Eigen::Matrix<double, 3, 1> temp;
    temp(0,0) = double((neighbour.x - source.x));
    temp(1,0) = double((neighbour.y - source.y));
    temp(2,0) = double((neighbour.z - source.z));

    double len = sqrt(
                temp(0, 0) * temp(0, 0) +
                temp(1, 0) * temp(1, 0) +
                temp(2, 0) * temp(2, 0)
     );

    if(len == 0.0)
        return false;

    (*V)(0, 0) = temp(0, 0);
    (*V)(1, 0) = temp(1, 0);
    (*V)(2, 0) = temp(2, 0);

    return true;
}

TensorType Descriptor::Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight)
{
    double norm, coeff, s, t;
    Eigen::Matrix3d vv(3,3), voteTemp(3,3);
    TensorType ballTensorVote;
    Eigen::Matrix3d I(3, 3);
    char* pEnd;
    float _sigma = ::strtof(_config->GetValue("sigma").c_str(), &pEnd);

    std::cout<<"SIGMA "<< _sigma<<std::endl;
    voteTemp.setZero(3, 3);
    vv.setZero(3, 3);
    I.setIdentity();

    s = V.norm(); // Euclidean Norm

    t = (s*s)/(_sigma * _sigma);

    coeff = exp(-1.0 * t);

    *weight += coeff;

    if(V.norm() != 0.0)
       V = V.normalized();

    norm = V.transpose() * V;
    norm = abs(norm);

    if(norm != 0.0)
    {
        vv = V * V.transpose();
        vv = vv / norm;
    }

    voteTemp = coeff * (I - vv);

    ballTensorVote.evec0[0] = voteTemp(0, 0);
    ballTensorVote.evec0[1] = voteTemp(0, 1);
    ballTensorVote.evec0[2] = voteTemp(0, 2);

    ballTensorVote.evec1[0] = voteTemp(1, 0);
    ballTensorVote.evec1[1] = voteTemp(1, 1);
    ballTensorVote.evec1[2] = voteTemp(1, 2);

    ballTensorVote.evec2[0] = voteTemp(2, 0);
    ballTensorVote.evec2[1] = voteTemp(2, 1);
    ballTensorVote.evec2[2] = voteTemp(2, 2);

    return ballTensorVote;
}

/*End the private method region */
