#ifndef TENSOR3DVOTINGCLASSIFIER_H
#define TENSOR3DVOTINGCLASSIFIER_H
#include "ClassifiersBase.h"

class Tensor3DVotingClassifier : public ClassifiersBase
{
public:
    Tensor3DVotingClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config;
    void Tensor3DVoting(float radius, std::vector<TensorType>& tensor3DVotins);
    glyphVars EigenDecomposition(TensorType tensor);
    void ComputeSaliencyVals(glyphVars& glyph);
    void glyphAnalysis(glyphVars& glyph);
    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors);
};

#endif // TENSOR3DVOTINGCLASSIFIER_H
