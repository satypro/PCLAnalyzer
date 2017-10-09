#ifndef DESCRIPTORPROCESSOR_H
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "result.h"
#include "Neighbours/types.h"
#include "descriptorprocessorbase.h"
#include "Neighbours/searchneighbourbase.h"
#define DESCRIPTORPROCESSOR_H

class DescriptorProcessor : public DescriptorProcessorBase
{
public:
    DescriptorProcessor(SearchNeighbourBase* searchNeighbourBase)
    {
        _searchNeighbour = searchNeighbourBase;
    }

    ~DescriptorProcessor();
    std::vector<Result*> getDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, SearchScale searchScale, NeighbourSearchTypes searchType);
private:
    SearchNeighbourBase* _searchNeighbour;
};

#endif // DESCRIPTORPROCESSOR_H
