#include "Neighbours/searchneighbour.h"
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Neighbours/types.h"
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbour::GetNeighbourCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ &searchPoint, SearchScale searchScale, NeighbourSearchTypes searchType)
{
    _octree = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(searchScale.resolution);
    _octree->setInputCloud (cloud);
    _octree->addPointsFromInputCloud ();

    switch(searchType)
    {
        case Radius:
            return GetRadiusSearchNeighbour(cloud, searchPoint, searchScale.radius);
            break;
        defualt:
            return cloud;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SearchNeighbour::GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, float radius)
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
