#ifndef SEARCHNEIGBOUR_H
#define SEARCHNEIGBOUR_H


#inlcude <pcl/point_types.h>
#include "types.h"

class SearchNeigbour
{
public:
    SearchNeigbour();
    pcl::PointCloud<pcl::PointXYZ>::Ptr
        GetNeighbourCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& searchPoint, SearchScale searchScale, NeighbourSearchTypes searchType);

};


#endif // SEARCHNEIGBOUR_H











