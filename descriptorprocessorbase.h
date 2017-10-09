#ifndef DESCRIPTORPROCESSORBASE
#define DESCRIPTORPROCESSORBASE
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "result.h"
#include "Neighbours/types.h"
#include "Neighbours/searchneighbourbase.h"

class DescriptorProcessorBase
{
public:
    virtual std::vector<Result*> getDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, SearchScale searchScale, NeighbourSearchTypes searchType) = 0;
};

#endif // DESCRIPTORPROCESSORBASE
