#ifndef DIFFUSEDNORMALVOTINGCLASSIFIER_H
#define DIFFUSEDNORMALVOTINGCLASSIFIER_H
#include "ClassifiersBase.h"
#include "Descriptors/PointDescriptor.h"

class DiffusedNormalVotingClassifier : public ClassifiersBase
{
public:
    DiffusedNormalVotingClassifier();
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
    glyphVars EigenDecomposition(TensorType tensor);
    void computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor);
    void glyphAnalysis(glyphVars& glyph);
};

#endif // DIFFUSEDNORMALVOTINGCLASSIFIER_H
