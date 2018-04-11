#ifndef TENSOR2DCLASSIFIER_H
#define TENSOR2DCLASSIFIER_H
#include "ClassifiersBase.h"

class Tensor2DClassifier : public ClassifiersBase
{
public:
    Tensor2DClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    void CalculatePartialDerivative(float radius, std::vector<Derivatives>& derivatives);
    void Tensor2D(float radius, std::vector<TensorType>& tensor2Ds);
    glyphVars EigenDecomposition(TensorType tensor);
    void ComputeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor)
};

#endif // TENSOR2DCLASSIFIER_H
