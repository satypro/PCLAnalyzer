#ifndef SEARCHNEIGHBOURBASE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include "SearchOptions.h"
#include "Config/Configuration.h"

#define SEARCHNEIGHBOURBASE

class SearchNeighbourBase
{
public:
    virtual void Build(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) = 0;
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetNeighbourCloud(pcl::PointXYZ& searchPoint) = 0;
    virtual std::vector<int> GetNeighbourCloudIndex() = 0;
    virtual Configuration* GetConfig() = 0;
    SearchOption searchOption;
};

#endif // SEARCHNEIGHBOURBASE
