#ifndef BOUNDARYTENSORCLASSIFIER_H
#define BOUNDARYTENSORCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

class BoundaryTensorClassifier : public ClassifiersBase
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
    BoundaryTensorClassifier();
    std::vector<IPointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }
private:
    Configuration* _config; 
    SearchNeighbourBase* _searchNeighbour;
    void CalculatePartialDerivative(float radius, std::vector<Derivatives>& derivatives);
    void CalculateSecondDerivative(float radius, std::vector<Derivatives>& derivatives);
    void CalculateThirdDerivative(float radius, std::vector<Derivatives>& derivatives);
    void BuildBoundaryTensor(float radius, std::vector<TensorType>& boundaryTensors);
    glyphVars EigenDecomposition(TensorType boundaryTensor);
    void ComputeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void GlyphAnalysis(glyphVars& glyph);
    void MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& boundaryTensor);
};

#endif // BOUNDARYTENSORCLASSIFIER_H
