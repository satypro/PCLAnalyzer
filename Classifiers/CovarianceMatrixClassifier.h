#ifndef COVARIANCEMATRIXCLASSIFIER_H
#define COVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

class CovarianceMatrixClassifier : public ClassifiersBase
{
public:
    CovarianceMatrixClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    SearchNeighbourBase* _searchNeighbour;
    void GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors);
    glyphVars EigenDecomposition(TensorType tensor);
    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor);
    void computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
};

#endif // COVARIANCEMATRIXCLASSIFIER_H
