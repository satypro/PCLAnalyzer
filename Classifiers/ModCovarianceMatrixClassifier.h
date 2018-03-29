#ifndef MODCOVARIANCEMATRIXCLASSIFIER_H
#define MODCOVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"

class ModCovarianceMatrixClassifier : public ClassifiersBase
{
public:
    ModCovarianceMatrixClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // MODCOVARIANCEMATRIXCLASSIFIER_H
