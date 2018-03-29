#ifndef BOUNDARYTENSORCLASSIFIER_H
#define BOUNDARYTENSORCLASSIFIER_H
#include "ClassifiersBase.h"

class BoundaryTensorClassifier : public ClassifiersBase
{
public:
    BoundaryTensorClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // BOUNDARYTENSORCLASSIFIER_H
