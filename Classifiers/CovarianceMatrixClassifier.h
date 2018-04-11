#ifndef COVARIANCEMATRIXCLASSIFIER_H
#define COVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"

class CovarianceMatrixClassifier : public ClassifiersBase
{
public:
    CovarianceMatrixClassifier();
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

    void GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors);
    glyphVars EigenDecomposition(TensorType tensor);
    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor);
    void ComputeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void GlyphAnalysis(glyphVars& glyph);
};

#endif // COVARIANCEMATRIXCLASSIFIER_H
