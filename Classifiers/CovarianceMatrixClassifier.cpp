#include "CovarianceMatrixClassifier.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include "Utilities/eig3.h"

CovarianceMatrixClassifier::CovarianceMatrixClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* CovarianceMatrixClassifier::GetConfig()
{
    return _config;
}

IPointDescriptor* CovarianceMatrixClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;
    _searchNeighbour = GetSearchStrategy();

    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    // float radius = ::strtof(_config->GetValue("radius").c_str(), &pEnd);
    float _scale = ::strtof(_config->GetValue("scale").c_str(), &pEnd);

    if(_scale == 0.0 || _rmin == 0.0 || _rmax == 0.0 || _rmin >= _rmax || getCloud()->points.size() == 0)
    {
        std::cout<<"invalid configuration parameters for classification module"<<std::endl;
        return pointDescriptor;
    }

    float dDeltaRadius = (_rmax - _rmin)/(_scale - 1.0);
    float radius = _rmin;

    TensorType averaged_tensor;
    TensorType covarianceTensor;

    while(radius <= _rmax)
    {
        covarianceTensor = GetCoVaraianceTensor(radius);
        MeasureProbability(pointDescriptor, averaged_tensor, covarianceTensor);
        radius += dDeltaRadius;
    }

    for(int j=0;j<3;j++)
    {
        averaged_tensor.evec0[j] /= _scale;
        averaged_tensor.evec1[j] /= _scale;
        averaged_tensor.evec2[j] /= _scale;
    }

    // We will swap the averaged_tensor and covarianceTensor
    MeasureProbability(pointDescriptor, covarianceTensor, averaged_tensor);

    return pointDescriptor;
}

TensorType CovarianceMatrixClassifier::GetCoVaraianceTensor(float radius)
{
    pcl::PointXYZ pointxyz = getSource();

    _searchNeighbour->searchOption.searchParameter.radius = radius;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud = _searchNeighbour->GetNeighbourCloud(pointxyz);

    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*_neighbourCloud, xyz_centroid);

    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix(*_neighbourCloud, xyz_centroid, covariance_matrix);

    TensorType covarianceTensor;

    covarianceTensor.evec0[0] = covariance_matrix(0, 0);
    covarianceTensor.evec0[1] = covariance_matrix(0, 1);
    covarianceTensor.evec0[2] = covariance_matrix(0, 2);

    covarianceTensor.evec1[0] = covariance_matrix(1, 0);
    covarianceTensor.evec1[1] = covariance_matrix(1, 1);
    covarianceTensor.evec1[2] = covariance_matrix(1, 2);

    covarianceTensor.evec2[0] = covariance_matrix(2, 0);
    covarianceTensor.evec2[1] = covariance_matrix(2, 1);
    covarianceTensor.evec2[2] = covariance_matrix(2, 2);

    return covarianceTensor;
}

void CovarianceMatrixClassifier::MeasureProbability(
                                                    PointDescriptor* pointDescriptor,
                                                    TensorType& averaged_tensor,
                                                    TensorType& covarianceTensor
)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);

    glyphVars glyph = EigenDecomposition(covarianceTensor);
    computeSaliencyVals(glyph, averaged_tensor);

    if(glyph.evals[2] == 0.0 && glyph.evals[1] == 0.0 && glyph.evals[0] == 0.0)
            {
                 pointDescriptor->featNode.prob[0] = pointDescriptor->featNode.prob[0] + 1;
            }
            else
            {
                if(glyph.evals[2] >= _epsi * glyph.evals[0]) //ev0>ev1>ev2
                    pointDescriptor->featNode.prob[0] = pointDescriptor->featNode.prob[0] + 1;

                if(glyph.evals[1] < _epsi * glyph.evals[0]) //ev0>ev1>ev2
                   pointDescriptor->featNode.prob[1] = pointDescriptor->featNode.prob[1] + 1;

                if(glyph.evals[2] < _epsi * glyph.evals[0])  //ev0>ev1>ev2
                    pointDescriptor->featNode.prob[2] = pointDescriptor->featNode.prob[2] + 1;


                pointDescriptor->featNode.featStrength[0] += ((glyph.evals[2] * glyph.evals[1])/(glyph.evals[0] * glyph.evals[0]));
                pointDescriptor->featNode.featStrength[1] += ((glyph.evals[2] * (glyph.evals[0] - glyph.evals[1]))/(glyph.evals[0] * glyph.evals[1]));
                pointDescriptor->featNode.featStrength[2] +=  glyph.evals[2] /(glyph.evals[0] + glyph.evals[1] + glyph.evals[2]);

            }


            pointDescriptor->featNode.csclcp[0] = glyph.csclcp[0];
            pointDescriptor->featNode.csclcp[1] = glyph.csclcp[1];
            pointDescriptor->featNode.csclcp[2] = glyph.csclcp[2];

            pointDescriptor->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
            if(glyph.evals[0] != 0)
            {
                pointDescriptor->featNode.planarity = (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
                pointDescriptor->featNode.anisotropy = (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
                pointDescriptor->featNode.sphericity = (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];
            }
            else
            {
                pointDescriptor->featNode.planarity = 0;
                pointDescriptor->featNode.anisotropy = 0;
                pointDescriptor->featNode.sphericity = 0;
            }


            if(glyph.evals[2] == 0.0 && glyph.evals[1] == 0.0 && glyph.evals[0] == 0.0)
            {
                 pointDescriptor->featNode.prob[0] = pointDescriptor->featNode.prob[0] + 1;
            }
            else
            {
                if(glyph.evals[2] >= _epsi * glyph.evals[0]) //ev0>ev1>ev2
                    pointDescriptor->featNode.prob[0] = pointDescriptor->featNode.prob[0] + 1;

                if(glyph.evals[1] < _epsi * glyph.evals[0]) //ev0>ev1>ev2
                   pointDescriptor->featNode.prob[1] = pointDescriptor->featNode.prob[1] + 1;

                if(glyph.evals[2] < _epsi * glyph.evals[0])  //ev0>ev1>ev2
                    pointDescriptor->featNode.prob[2] = pointDescriptor->featNode.prob[2] + 1;


                pointDescriptor->featNode.featStrength[0] += ((glyph.evals[2] * glyph.evals[1])/(glyph.evals[0] * glyph.evals[0]));
                pointDescriptor->featNode.featStrength[1] += ((glyph.evals[2] * (glyph.evals[0] - glyph.evals[1]))/(glyph.evals[0] * glyph.evals[1]));
                pointDescriptor->featNode.featStrength[2] +=  glyph.evals[2] /(glyph.evals[0] + glyph.evals[1] + glyph.evals[2]);

            }

            pointDescriptor->featNode.csclcp[0] = glyph.csclcp[0];
            pointDescriptor->featNode.csclcp[1] = glyph.csclcp[1];
            pointDescriptor->featNode.csclcp[2] = glyph.csclcp[2];

            pointDescriptor->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
            if(glyph.evals[0] != 0)
            {
                pointDescriptor->featNode.planarity = (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
                pointDescriptor->featNode.anisotropy = (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
                pointDescriptor->featNode.sphericity = (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];
            }
            else
            {
                pointDescriptor->featNode.planarity = 0;
                pointDescriptor->featNode.anisotropy = 0;
                pointDescriptor->featNode.sphericity = 0;
            }
}

glyphVars CovarianceMatrixClassifier::EigenDecomposition(TensorType tensor)
{
    glyphVars glyph;

    float A[3][3], V[3][3], d[3];

    for(int i = 0; i < 3; i++)
    {
        A[i][0] = tensor.evec0[i];
        A[i][1] = tensor.evec1[i];
        A[i][2] = tensor.evec2[i];
    }

    eigen_decomposition(A, V, d);

    glyph.evals[2] = d[0] ;   //d[2] > d[1] > d[0]
    glyph.evals[1] = d[1] ;
    glyph.evals[0] = d[2] ;

    glyph.evecs[0] = V[0][2];
    glyph.evecs[1] = V[1][2];
    glyph.evecs[2] = V[2][2];

    glyph.evecs[3] = V[0][1];
    glyph.evecs[4] = V[1][1];
    glyph.evecs[5] = V[2][1];

    glyph.evecs[6] = V[0][0];
    glyph.evecs[7] = V[1][0];
    glyph.evecs[8] = V[2][0];
}

void CovarianceMatrixClassifier::computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor)
{
    float len = glyph.evals[2] + glyph.evals[1] + glyph.evals[0];

    float cl = 0.0, cp = 0.0, cs = 0.0;

    if(len!= 0.0)
    {
        cl = (glyph.evals[0] - glyph.evals[1])/len; //ev0>ev1>ev2
        cp = (2*(glyph.evals[1] - glyph.evals[2]))/len ;//(2.0 * (eigen_values[1] - eigen_values[0]));
        cs = 1 - (cl+cp); //1.0 - cl - cp;

        // lamda0>lambda1>lambda2
        float lamda0 = glyph.evals[0] / len;
        float lamda1 = glyph.evals[1] / len;
        float lamda2 = glyph.evals[2] / len;
        Eigen::MatrixXf e0(3,1);
        e0 << glyph.evecs[0],glyph.evecs[1],glyph.evecs[2];
        e0.normalize();
        Eigen::MatrixXf e1(3,1);
        e1 << glyph.evecs[3],glyph.evecs[4],glyph.evecs[5];
        e1.normalize();
        Eigen::MatrixXf e2(3,1);
        e2 << glyph.evecs[6],glyph.evecs[7],glyph.evecs[8];
        e2.normalize();
        Eigen::Matrix3f T;
        T << 0,0,0,0,0,0,0,0,0;

        T += lamda0 * e0 * e0.transpose();
        T += lamda1 * e1 * e1.transpose();
        T += lamda2 * e2 * e2.transpose();

        averaged_tensor.evec0[0] += T(0,0);
        averaged_tensor.evec0[1] += T(0,1);
        averaged_tensor.evec0[2] += T(0,2);

        averaged_tensor.evec0[3] += T(1,0);
        averaged_tensor.evec0[4] += T(1,1);
        averaged_tensor.evec0[5] += T(1,2);

        averaged_tensor.evec0[6] += T(2,0);
        averaged_tensor.evec0[7] += T(2,1);
        averaged_tensor.evec0[8] += T(2,2);
    }

    glyph.csclcp[1] = cl;
    glyph.csclcp[0] = cs;
    glyph.csclcp[2] = cp;
}

void CovarianceMatrixClassifier::glyphAnalysis(glyphVars& glyph)
{

}
