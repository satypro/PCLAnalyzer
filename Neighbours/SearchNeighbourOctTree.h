#ifndef SEARCHNEIGHBOUR_H
#define SEARCHNEIGHBOUR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include "SearchOptions.h"
#include "SearchNeighbourBase.h"

class SearchNeighbourOctTree : public SearchNeighbourBase
{
public:
    SearchNeighbourOctTree();
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetNeighbourCloud(pcl::PointXYZ& searchPoint, SearchOption option);
    void Build(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float resolution);
    Configuration* GetConfig();

private:
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* _octree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetRadiusSearchNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, float radius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        SearchKNearest(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint, int k);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        SearchVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ& searchPoint);
    Configuration* _config;
};

#endif // SEARCHNEIGHBOUR_H
