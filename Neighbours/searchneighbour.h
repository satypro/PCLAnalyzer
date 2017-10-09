#ifndef SEARCHNEIGHBOUR_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include "types.h"
#include "searchneighbourbase.h"
#define SEARCHNEIGHBOUR_H

class SearchNeighbour : public SearchNeighbourBase
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetNeighbourCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& searchPoint, SearchScale searchScale, NeighbourSearchTypes searchType);

private:
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* _octree;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, float radius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        SearchKNearest(pcl::PointXYZ& searchPoint, int k);
};

#endif // SEARCHNEIGHBOUR_H
