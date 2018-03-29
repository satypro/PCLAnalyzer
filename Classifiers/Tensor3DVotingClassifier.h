#ifndef TENSOR3DVOTINGCLASSIFIER_H
#define TENSOR3DVOTINGCLASSIFIER_H
#include "ClassifiersBase.h"

class Tensor3DVotingClassifier : public ClassifiersBase
{
public:
    Tensor3DVotingClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // TENSOR3DVOTINGCLASSIFIER_H
