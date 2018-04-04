#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include "ClassifiersBase.h"

class Classifier : public ClassifiersBase
{
public:
    Classifier();
    IPointDescriptor* Classify();
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
    glyphVars EigenDecomposition(TensorType tensor);
    void getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel);
    void computeSaliencyVals(glyphVars& glyph);
    void glyphAnalysis(glyphVars& glyph);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud;
};

#endif // CLASSIFIER_H
