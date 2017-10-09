#include <iostream>
#include <ctime>
#include <vector>
#include <string>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "descriptorprocessor.h"
#include "Neighbours/searchneighbour.h"
#include "Neighbours/types.h"
#include "result.h"
#include "datatypes.h"

std::vector<Result*> DescriptorProcessor::getDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, SearchScale searchScale, NeighbourSearchTypes searchType)
{
    std::vector<Result*> result (cloud->points.size());

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ searchPoint = cloud->points[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour = _searchNeighbour->
                GetNeighbourCloud(cloud, searchPoint, searchScale, searchType);

        // Result
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        tensorType tensor;

        pcl::compute3DCentroid(*neighbour, xyz_centroid);
        pcl::computeCovarianceMatrix(*neighbour, xyz_centroid, covariance_matrix);

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

        Result* res = new Result();
        res->Centroid = xyz_centroid;
        res->CovarianceMatrix = covariance_matrix;
        res->Tensor = tensor;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
        if (eigensolver.info() != Eigen::Success)
        {
            res->EigenValues = eigensolver.eigenvalues();
            res->EigenVectors = eigensolver.eigenvectors();
        }
        else
        {
            res->EigenValues = eigensolver.eigenvalues();
            res->EigenVectors = eigensolver.eigenvectors();
        }

        result[i] = res;

        // this search point now start getting the descriptiors like covariance matrix, Tensor voting etc....
        //std::cout<<"Generating Descriptors For Cloud Point : EigenValues "<< res->EigenValues <<std::endl;
        //std::cout<<"Generating Descriptors For Cloud Point : EigenVector "<< res->EigenVectors <<std::endl;
        std::cout<<"Generating Descriptors Progress : "<<(i / cloud->points.size ())*100<<"%"<<std::endl;
     }

     return result;
}
