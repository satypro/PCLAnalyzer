#include "Barycentric.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include "Utilities/eig3.h"
#include <math.h>

Barycentric::Barycentric()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* Barycentric::GetConfig()
{
    return _config;
}

std::vector<PointDescriptor*> Barycentric::Classify()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = getCloud();
    size_t cloudSize = cloud->points.size();
    std::vector<PointDescriptor*> descriptors(cloudSize, new PointDescriptor());

    char* pEnd;
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    float _scale = ::strtof(_config->GetValue("scale").c_str(), &pEnd);

    std::vector<int> optimalScales;

    for(int index = 0 ; index < cloudSize; index++)
    {
        if (isnan(cloud->points[index].x) || isnan(cloud->points[index].y) || isnan(cloud->points[index].z))
        {
            std::cout<<"The Point at : "<<index<<" NAN : "<<std::endl;
            optimalScales.push_back(0);
            continue;
        }

        int optimalScale = GetOptimalScale(cloud->points[index]);

        // Now for this optimal scale again perform the Neighbour Search
        // And Evaluate cl, cp and cs

        _searchNeighbour->searchOption.searchParameter.kNearest = optimalScale;
        _neighbourCloud = _searchNeighbour->GetNeighbourCloud(cloud->points[index]);
    }

    return descriptors;
}

int Barycentric::GetOptimalScale(pcl::PointXYZ& searchPoint)
{
    int optimalScale = 1;

    float S_Min = 1;

    for (int K = 10 ; K <= 100 ; K++ )
    {
        _searchNeighbour->searchOption.searchParameter.kNearest = K;
        _neighbourCloud = _searchNeighbour->GetNeighbourCloud(searchPoint);

        //Now for the neighbour cloud evaluate its CoVariance Matrix and Eigen decomposition
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid(*_neighbourCloud, xyz_centroid);

        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix(*_neighbourCloud, xyz_centroid, covariance_matrix);

        TensorType tensor;

        tensor.evec0[0] = covariance_matrix(0, 0);
        tensor.evec0[1] = covariance_matrix(0, 1);
        tensor.evec0[2] = covariance_matrix(0, 2);

        tensor.evec1[0] = covariance_matrix(1, 0);
        tensor.evec1[1] = covariance_matrix(1, 1);
        tensor.evec1[2] = covariance_matrix(1, 2);

        tensor.evec2[0] = covariance_matrix(2, 0);
        tensor.evec2[1] = covariance_matrix(2, 1);
        tensor.evec2[2] = covariance_matrix(2, 2);

        // Now perform the eigen decomposition.
        glyphVars glyph = EigenDecomposition(tensor);

        // As per WeinMann Paper 2014 Paper
        // using the eigen value to calculate the shannon entropy.

        // lets normalized the eigen values;

        float eigenValueSum = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];

        float e1 = glyph.evals[0] / eigenValueSum;
        float e2 = glyph.evals[1] / eigenValueSum;
        float e3 = glyph.evals[2] / eigenValueSum;

        // calculate the Shannon Entropy;
        // As e1+e2+e3 = 1

        float S = -e1 * log(e1) - e2 * log(e2) - e3 * log(e3);

        // Also S >= 0 , since log(e1) , log(e2), log(e3) < 0
        if (S < S_Min)
        {
            S_Min = S;
            optimalScale = K;
        }
    }

    return optimalScale;
}

glyphVars Barycentric::EigenDecomposition(TensorType tensor)
{
    glyphVars glyph;

    float A[3][3], V[3][3], d[3];

    for(int i = 0; i < 3; i++)
    {
        A[i][0] = tensor.evec0[i];
        A[i][1] = tensor.evec1[i];
        A[i][2] = tensor.evec2[i];
    }

    eigen_decomposition(A, V, d);

    glyph.evals[2] = d[0] ;   //d[2] > d[1] > d[0]
    glyph.evals[1] = d[1] ;
    glyph.evals[0] = d[2] ;

    glyph.evecs[0] = V[0][2];
    glyph.evecs[1] = V[1][2];
    glyph.evecs[2] = V[2][2];

    glyph.evecs[3] = V[0][1];
    glyph.evecs[4] = V[1][1];
    glyph.evecs[5] = V[2][1];

    glyph.evecs[6] = V[0][0];
    glyph.evecs[7] = V[1][0];
    glyph.evecs[8] = V[2][0];
}
