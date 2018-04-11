#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include "ClassifiersBase.h"

class Classifier : public ClassifiersBase
{
public:
    Classifier();
    std::vector<PointDescriptor*> Classify();
    Configuration* GetConfig();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        ClassifiersBase::setCloud(cloud);
    }

private:
    Configuration* _config;
    Eigen::Vector4f Get3DCentroid();
    Eigen::Matrix3f ComputeCovarianceMatrix();
    TensorType GetCoVaraianceTensor();
    TensorType Get3DVotingTensor();

    bool MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V);
    TensorType Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight);
    void getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel);
    glyphVars EigenDecomposition(TensorType tensor);
    void computeSaliencyVals(glyphVars& glyph);
    void glyphAnalysis(glyphVars& glyph);
    PointDescriptor* Process();
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud;
};

#endif // CLASSIFIER_H
