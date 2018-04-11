#ifndef MODCOVARIANCEMATRIXCLASSIFIER_H
#define MODCOVARIANCEMATRIXCLASSIFIER_H
#include "ClassifiersBase.h"

class ModCovarianceMatrixClassifier : public ClassifiersBase
{
public:
    ModCovarianceMatrixClassifier();
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

    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors);
    void GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors);
    glyphVars EigenDecomposition(TensorType tensor);
    void ComputeSaliencyVals(glyphVars& glyph);
    void GlyphAnalysis(glyphVars& glyph);
};

#endif // MODCOVARIANCEMATRIXCLASSIFIER_H
