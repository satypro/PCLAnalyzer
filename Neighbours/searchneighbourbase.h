#ifndef SEARCHNEIGHBOURBASE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include "types.h"
#define SEARCHNEIGHBOURBASE

class SearchNeighbourBase
{
public:
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetNeighbourCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& searchPoint, SearchScale searchScale, NeighbourSearchTypes searchType) = 0;
};

#endif // SEARCHNEIGHBOURBASE
