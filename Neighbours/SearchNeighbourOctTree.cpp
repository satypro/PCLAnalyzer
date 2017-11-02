#include "Neighbours/SearchNeighbourOctTree.h"
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Neighbours/SearchOptions.h"
#include <iostream>

void SearchNeighbourOctTree::Build(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float resolution)
{
    _cloud = cloud;
    _octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution);
    _octree->setInputCloud (cloud);
    _octree->addPointsFromInputCloud ();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbourOctTree::GetNeighbourCloud(pcl::PointXYZ &searchPoint, SearchOption searchOption)
{
    switch(searchOption.neighbourSearchTypes)
    {
        case Radius:
            return GetRadiusSearchNeighbour(_cloud, searchPoint, searchOption.scale.radius);
            break;
        case KNearest:
            return SearchKNearest(_cloud, searchPoint, searchOption.scale.kNearest);
            break;
        case Voxel:
            return SearchVoxel(_cloud, searchPoint);
            break;
        defualt:
            return _cloud;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbourOctTree::GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_Cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (_octree->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        temp_Cloud->points.resize(pointIdxRadiusSearch.size());

        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            temp_Cloud->points[i] = cloud->points[ pointIdxRadiusSearch[i] ];
        }
    }

    return temp_Cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbourOctTree::SearchKNearest(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, int k)
{
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    if (_octree->nearestKSearch (searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            cloud_cluster->points.push_back(cloud->points[pointIdxNKNSearch[i]]);
        }
    }

    return cloud_cluster;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbourOctTree::SearchVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint)
{
    std::vector<int> pointIdxVec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    if (_octree->voxelSearch (searchPoint, pointIdxVec))
    {
        for (size_t i = 0; i < pointIdxVec.size (); ++i)
        {
            cloud_cluster->points.push_back(cloud->points[pointIdxVec[i]]);
        }
    }

    return cloud_cluster;
}