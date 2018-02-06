#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Types/datatypes.h"
#include "DescriptorBase.h"

class Descriptor : public DescriptorBase
{
public:
    Descriptor();
    Configuration* GetConfig();
    PointDescriptor GeneratePointDescriptor();

    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        DescriptorBase::setCloud(cloud);
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
};

#endif // DESCRIPTOR_H
