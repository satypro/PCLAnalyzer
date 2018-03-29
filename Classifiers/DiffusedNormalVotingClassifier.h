#ifndef DIFFUSEDNORMALVOTINGCLASSIFIER_H
#define DIFFUSEDNORMALVOTINGCLASSIFIER_H
#include "ClassifiersBase.h"

class DiffusedNormalVotingClassifier : public ClassifiersBase
{
public:
    DiffusedNormalVotingClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // DIFFUSEDNORMALVOTINGCLASSIFIER_H
