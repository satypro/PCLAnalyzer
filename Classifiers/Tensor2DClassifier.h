#ifndef TENSOR2DCLASSIFIER_H
#define TENSOR2DCLASSIFIER_H
#include "ClassifiersBase.h"

class Tensor2DClassifier : public ClassifiersBase
{
public:
    Tensor2DClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // TENSOR2DCLASSIFIER_H
