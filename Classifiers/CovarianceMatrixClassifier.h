#ifndef COVARIANCEMATRIXCLASSIFIER_H
#define COVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

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
    SearchNeighbourBase* _searchNeighbour;
    TensorType GetCoVaraianceTensor(float radius);
    void MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& covarianceTensor);
    glyphVars EigenDecomposition(TensorType tensor);
    void computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
};

#endif // COVARIANCEMATRIXCLASSIFIER_H
