#ifndef COVARIANCEMATRIXCLASSIFIER_H
#define COVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"

class CovarianceMatrixClassifier : public ClassifiersBase
{
public:
    CovarianceMatrixClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // COVARIANCEMATRIXCLASSIFIER_H
