#ifndef CLASSIFIERSBASE_H
#define CLASSIFIERSBASE_H
#include <vector>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Types/datatypes.h"
#include "ClassifierLabels.h"
#include "Config/Configuration.h"
#include "Descriptors/IPointDescriptor.h"

class ClassifiersBase
{
public:
    ClassifiersBase();
    virtual IPointDescriptor* Classify() = 0 ;
    virtual Configuration* GetConfig() = 0;

    void virtual setSource(pcl::PointXYZ sourcePoint)
    {
        _sourcePoint = sourcePoint;
    }

    pcl::PointXYZ virtual getSource()
    {
        return _sourcePoint;
    }

    void virtual setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        _cloud = cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr virtual getCloud()
    {
        return _cloud;
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::PointXYZ _sourcePoint;
};

#endif // CLASSIFIERSBASE_H
