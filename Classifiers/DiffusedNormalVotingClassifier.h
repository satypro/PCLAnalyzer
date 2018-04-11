#ifndef DIFFUSEDNORMALVOTINGCLASSIFIER_H
#define DIFFUSEDNORMALVOTINGCLASSIFIER_H
#include "ClassifiersBase.h"

class DiffusedNormalVotingClassifier : public ClassifiersBase
{
public:
    DiffusedNormalVotingClassifier();
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
    void GetTensor(float radius, std::vector<TensorType>& tensors);
    TensorType Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight);
    bool MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V);
    glyphVars EigenDecomposition(TensorType tensor);
    void Getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel);
    void ComputeSaliencyVals(glyphVars& glyph);
    void GlyphAnalysis(glyphVars& glyph);
};

#endif // DIFFUSEDNORMALVOTINGCLASSIFIER_H
