#ifndef TENSOR2DCLASSIFIER_H
#define TENSOR2DCLASSIFIER_H
#include "ClassifiersBase.h"

class Tensor2DClassifier : public ClassifiersBase
{
public:
    struct Derivatives
    {
        float Ix;
        float Iy;
        float Ixx;
        float Iyy;
        float Ixy;
        float Iyx;
        float Ixxx;
        float Ixxy;
        float Ixyy;
        float Iyyy;

        Derivatives(){}
    };

public:
    Tensor2DClassifier();
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

    void Process(std::vector<PointDescriptor*>& pointDescriptors,
                 std::vector<TensorType>& tensors);
    void CalculatePartialDerivative(float radius, std::vector<Derivatives>& derivatives);
    void Tensor2D(float radius, std::vector<TensorType>& tensor2Ds);
    glyphVars EigenDecomposition(TensorType tensor);
    void ComputeSaliencyVals(glyphVars& glyph);
    void GlyphAnalysis(glyphVars& glyph);
};

#endif // TENSOR2DCLASSIFIER_H
