#ifndef BARYCENTRIC_H
#define BARYCENTRIC_H
#include <vector>
#include "Descriptors/PointDescriptor.h"
#include "ClassifiersBase.h"

class Barycentric : public ClassifiersBase
{
public:
    Barycentric();
    std::vector<PointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    SearchNeighbourBase* _searchNeighbour;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud;
    int GetOptimalScale(pcl::PointXYZ& searchPoint);
    glyphVars EigenDecomposition(TensorType tensor);
};

#endif // BARYCENTRIC_H
