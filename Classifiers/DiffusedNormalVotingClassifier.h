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
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud;

    void Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors);
    void GetTensor(float radius, std::vector<TensorType>& tensors);
    TensorType Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight);
    bool MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V);
    glyphVars EigenDecomposition(TensorType tensor);
    void getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel);
    void computeSaliencyVals(glyphVars& glyph);
    void glyphAnalysis(glyphVars& glyph);
};

#endif // DIFFUSEDNORMALVOTINGCLASSIFIER_H
