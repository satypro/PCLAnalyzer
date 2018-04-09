#ifndef COVARIANCEMATRIX2DCLASSIFIER_H
#define COVARIANCEMATRIX2DCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

class CovarianceMatrix2DClassifier : public ClassifiersBase
{
public:
    CovarianceMatrix2DClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    SearchNeighbourBase* _searchNeighbour;
    IPointDescriptor* Process();
    TensorType GetCoVaraianceTensor(float radius);
    void MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& covarianceTensor);
    void computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
    glyphVars EigenDecomposition(TensorType tensor);
};

#endif // COVARIANCEMATRIX2DCLASSIFIER_H
