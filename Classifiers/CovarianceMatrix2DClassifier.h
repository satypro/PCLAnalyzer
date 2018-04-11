#ifndef COVARIANCEMATRIX2DCLASSIFIER_H
#define COVARIANCEMATRIX2DCLASSIFIER_H
#include "ClassifiersBase.h"

class CovarianceMatrix2DClassifier : public ClassifiersBase
{
public:
    CovarianceMatrix2DClassifier();
    std::vector<PointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    SearchNeighbourBase* _searchNeighbour;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud;

    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor);
    TensorType GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors);
    void MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& covarianceTensor);
    void ComputeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void GlyphAnalysis(glyphVars& glyph);
    glyphVars EigenDecomposition(TensorType tensor);
};

#endif // COVARIANCEMATRIX2DCLASSIFIER_H
