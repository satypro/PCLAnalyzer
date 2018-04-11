#ifndef MODCOVARIANCEMATRIXCLASSIFIER_H
#define MODCOVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

class ModCovarianceMatrixClassifier : public ClassifiersBase
{
public:
    ModCovarianceMatrixClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    SearchNeighbourBase* _searchNeighbour;
    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors);
    void GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors);
    glyphVars EigenDecomposition(TensorType tensor);
    void computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
};

#endif // MODCOVARIANCEMATRIXCLASSIFIER_H
