#ifndef COVARIANCEMATRIX2DCLASSIFIER_H
#define COVARIANCEMATRIX2DCLASSIFIER_H
#include "ClassifiersBase.h"

class CovarianceMatrix2DClassifier : public ClassifiersBase
{
public:
    CovarianceMatrix2DClassifier();
    IPointDescriptor* Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
};

#endif // COVARIANCEMATRIX2DCLASSIFIER_H
