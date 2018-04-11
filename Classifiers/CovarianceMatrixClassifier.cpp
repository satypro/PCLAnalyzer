#include "CovarianceMatrixClassifier.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <teem/ten.h>
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

std::vector<IPointDescriptor*> CovarianceMatrixClassifier::Classify()
{
    size_t cloudSize = getCloud()->points.size();
    std::vector<IPointDescriptor*> descriptors(cloudSize, new PointDescriptor());

    char* pEnd;
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    float _scale = ::strtof(_config->GetValue("scale").c_str(), &pEnd);

    /*if(_scale == 0.0 || _rmin == 0.0 || _rmax == 0.0 || _rmax >= _rmin || getCloud()->points.size() == 0)
    {
        std::cout<<"invalid configuration parameters for classification module"<<std::endl;
        return descriptors;
    }*/

    float dDeltaRadius = (_rmax - _rmin)/(_scale - 1.0);
    float radius = _rmin;

    while(radius <= _rmax)
    {
        std::vector<TensorType> tensors(cloudSize, TensorType());
        std::vector<TensorType> averaged_tensor(cloudSize, TensorType())
        GetCoVaraianceTensor(radius, tensors);
        Process(descriptors, tensors, averaged_tensor);
        radius += dDeltaRadius;
    }
}

void CovarianceMatrixClassifier::Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);

    for (int i = 0; getCloud()->points.size(); i++)
    {
        glyphVars glyph = EigenDecomposition(tensors[i]);
        computeSaliencyVals(glyph, averaged_tensor[i]);
        glyphAnalysis(glyph);

        pointDescriptors[i]->glyph = glyph;

        if(glyph.evals[2] == 0.0 && glyph.evals[1] == 0.0 && glyph.evals[0] == 0.0)
        {
            pointDescriptors[i]->featNode.prob[0] = pointDescriptors[i]->featNode.prob[0] + 1;
        }
        else
        {
            if(glyph.evals[2] >= _epsi * glyph.evals[0]) //ev0>ev1>ev2
                pointDescriptors[i]->featNode.prob[0] = pointDescriptors[i]->featNode.prob[0] + 1;

            if(glyph.evals[1] < _epsi * glyph.evals[0]) //ev0>ev1>ev2
            pointDescriptors[i]->featNode.prob[1] = pointDescriptors[i]->featNode.prob[1] + 1;

            if(glyph.evals[2] < _epsi * glyph.evals[0])  //ev0>ev1>ev2
                pointDescriptors[i]->featNode.prob[2] = pointDescriptors[i]->featNode.prob[2] + 1;


            pointDescriptors[i]->featNode.featStrength[0] += ((glyph.evals[2] * glyph.evals[1])/(glyph.evals[0] * glyph.evals[0]));
            pointDescriptors[i]->featNode.featStrength[1] += ((glyph.evals[2] * (glyph.evals[0] - glyph.evals[1]))/(glyph.evals[0] * glyph.evals[1]));
            pointDescriptors[i]->featNode.featStrength[2] +=  glyph.evals[2] /(glyph.evals[0] + glyph.evals[1] + glyph.evals[2]);
        }

        pointDescriptors[i]->featNode.csclcp[0] = glyph.csclcp[0];
        pointDescriptors[i]->featNode.csclcp[1] = glyph.csclcp[1];
        pointDescriptors[i]->featNode.csclcp[2] = glyph.csclcp[2];

        pointDescriptors[i]->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
        if(glyph.evals[0] != 0)
        {
            pointDescriptors[i]->featNode.planarity = (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
            pointDescriptors[i]->featNode.anisotropy = (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
            pointDescriptors[i]->featNode.sphericity = (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];
        }
        else
        {
            pointDescriptors[i]->featNode.planarity = 0;
            pointDescriptors[i]->featNode.anisotropy = 0;
            pointDescriptors[i]->featNode.sphericity = 0;
        }

        if(glyph.evals[2] == 0.0 && glyph.evals[1] == 0.0 && glyph.evals[0] == 0.0)
        {
            pointDescriptors[i]->featNode.prob[0] = pointDescriptors[i]->featNode.prob[0] + 1;
        }
        else
        {
            if(glyph.evals[2] >= _epsi * glyph.evals[0]) //ev0>ev1>ev2
                pointDescriptors[i]->featNode.prob[0] = pointDescriptors[i]->featNode.prob[0] + 1;

            if(glyph.evals[1] < _epsi * glyph.evals[0]) //ev0>ev1>ev2
            pointDescriptors[i]->featNode.prob[1] = pointDescriptors[i]->featNode.prob[1] + 1;

            if(glyph.evals[2] < _epsi * glyph.evals[0])  //ev0>ev1>ev2
                pointDescriptors[i]->featNode.prob[2] = pointDescriptors[i]->featNode.prob[2] + 1;

            pointDescriptors[i]->featNode.featStrength[0] += ((glyph.evals[2] * glyph.evals[1])/(glyph.evals[0] * glyph.evals[0]));
            pointDescriptors[i]->featNode.featStrength[1] += ((glyph.evals[2] * (glyph.evals[0] - glyph.evals[1]))/(glyph.evals[0] * glyph.evals[1]));
            pointDescriptors[i]->featNode.featStrength[2] +=  glyph.evals[2] /(glyph.evals[0] + glyph.evals[1] + glyph.evals[2]);
        }

        pointDescriptors[i]->featNode.csclcp[0] = glyph.csclcp[0];
        pointDescriptors[i]->featNode.csclcp[1] = glyph.csclcp[1];
        pointDescriptors[i]->featNode.csclcp[2] = glyph.csclcp[2];

        pointDescriptors[i]->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
        if(glyph.evals[0] != 0)
        {
            pointDescriptors[i]->featNode.planarity = (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
            pointDescriptors[i]->featNode.anisotropy = (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
            pointDescriptors[i]->featNode.sphericity = (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];
        }
        else
        {
            pointDescriptors[i]->featNode.planarity = 0;
            pointDescriptors[i]->featNode.anisotropy = 0;
            pointDescriptors[i]->featNode.sphericity = 0;
        }
    }
}

void CovarianceMatrixClassifier::GetCoVaraianceTensor(float radius, std::vector<TensorType>& tensors)
{
    int index = -1;
    _searchNeighbour->searchOption.searchParameter.radius = radius;

    for(pcl::PointXYZ searchPoint : getCloud())
    {
        index++;

        for(int j =0; j < 3; j++)
        {
            tensors[index].evec0[j] = 0;
            tensors[index].evec1[j] = 0;
            tensors[index].evec2[j] = 0;
        }

        _neighbourCloud = _searchNeighbour->GetNeighbourCloud(searchPoint);
        
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid(*_neighbourCloud, xyz_centroid);

        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix(*_neighbourCloud, xyz_centroid, covariance_matrix);

        tensors[index].evec0[0] = covariance_matrix(0, 0);
        tensors[index].evec0[1] = covariance_matrix(0, 1);
        tensors[index].evec0[2] = covariance_matrix(0, 2);

        tensors[index].evec1[0] = covariance_matrix(1, 0);
        tensors[index].evec1[1] = covariance_matrix(1, 1);
        tensors[index].evec1[2] = covariance_matrix(1, 2);

        tensors[index].evec2[0] = covariance_matrix(2, 0);
        tensors[index].evec2[1] = covariance_matrix(2, 1);
        tensors[index].evec2[2] = covariance_matrix(2, 2);
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
       double eps=1e-4;
       double evals[3], uv[2], abc[3];

       double norm = sqrt(glyph.evals[2] * glyph.evals[2] + glyph.evals[1]*glyph.evals[1] + glyph.evals[0] *glyph.evals[0]);

       evals[0] = glyph.evals[2];   //ev0>ev1>ev2
       evals[1] = glyph.evals[1];
       evals[2] = glyph.evals[0];

       //tenGlyphBqdUvEval(uv, evals);
       //tenGlyphBqdAbcUv(abc, uv, 3.0);

       norm=ELL_3V_LEN(evals);

       if (norm<eps)
       {
         double weight=norm/eps;
         abc[0]=weight*abc[0]+(1-weight);
         abc[1]=weight*abc[1]+(1-weight);
         abc[2]=weight*abc[2]+(1-weight);
       }

       glyph.uv[0] = uv[0];
       glyph.uv[1] = uv[1];

       glyph.abc[0] = abc[0];
       glyph.abc[1] = abc[1];
       glyph.abc[2] = abc[2];
}
