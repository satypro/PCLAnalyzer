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
#include "Neighbours/searchneighbour.h"
#include "cloudprocessor.h"
#include "Neighbours/types.h"
#include "result.h"
#include "datatypes.h"

CloudProcessor::CloudProcessor()
{
}



std::vector<Result*> getDescriptors(std::string fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
            return std::vector<Result*>();
    }

    std::vector<Result*> result (cloud->points.size());
    SearchScale scale;
    scale.radius = 0.5f;
    scale.resolution = 128.0f;

    SearchNeighbour* search =  new SearchNeighbour();

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ searchPoint = cloud->points[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour = search->
                GetNeighbourCloud(cloud, searchPoint, scale, Radius);

        // Result
        Eigen::Vector4f xyz_centroid;
        Eigen::Matrix3f covariance_matrix;
        tensorType tensor;

        pcl::compute3DCentroid(*neighbour, xyz_centroid);
        pcl::computeCovarianceMatrix(*neighbour, xyz_centroid, covariance_matrix);

        std::cout<<"Generating Descriptors For Cloud Point : "<< i <<std::endl;

        for (int i = 0 ; i < 3 ; i++)
        {
            for (int j = 0 ; j < 3; j++)
            {
                if (i == 0)
                    tensor.evec0[j] = covariance_matrix(i, j);
                if (i == 1)
                    tensor.evec1[j] = covariance_matrix(i, j);
                if (i == 2)
                    tensor.evec2[j] = covariance_matrix(i, j);
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

        std::cout<<"Generating Descriptors For Cloud Point : EigenValues "<< res->EigenValues <<std::endl;
        std::cout<<"Generating Descriptors For Cloud Point : EigenVector "<< res->EigenVectors <<std::endl;

        result[i] = res;

        // this search point now start getting the descriptiors like covariance matrix, Tensor voting etc....
     }

     return result;
}
