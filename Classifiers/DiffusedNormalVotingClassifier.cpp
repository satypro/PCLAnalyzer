#include "DiffusedNormalVotingClassifier.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <teem/ten.h>
#include "Utilities/CommonUtility.h"
#include "Utilities/eig3.h"
#include <iostream>

DiffusedNormalVotingClassifier::DiffusedNormalVotingClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* DiffusedNormalVotingClassifier::GetConfig()
{
    return _config;
}

std::vector<PointDescriptor*> DiffusedNormalVotingClassifier::Classify()
{
    size_t cloudSize = getCloud()->points.size();
    std::vector<PointDescriptor*> descriptors(cloudSize, new PointDescriptor());
    _searchNeighbour = GetSearchStrategy();
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
        GetTensor(radius, tensors);
        Process(descriptors, tensors);
        radius += dDeltaRadius;
    }

    for(int i = 0 ; i < descriptors.size() ; i++)
    {
        descriptors[i]->featNode.prob[0] = descriptors[i]->featNode.prob[0]/_scale;
        descriptors[i]->featNode.prob[1] = descriptors[i]->featNode.prob[1]/_scale;
        descriptors[i]->featNode.prob[2] = descriptors[i]->featNode.prob[2]/_scale;

        descriptors[i]->featNode.featStrength[0] = descriptors[i]->featNode.featStrength[0]/_scale;
        descriptors[i]->featNode.featStrength[1] = descriptors[i]->featNode.featStrength[1]/_scale;
        descriptors[i]->featNode.featStrength[2] = descriptors[i]->featNode.featStrength[2]/_scale;

        descriptors[i]->featNode.csclcp[0] = descriptors[i]->featNode.csclcp[0]/_scale;
        descriptors[i]->featNode.csclcp[1] = descriptors[i]->featNode.csclcp[1]/_scale;
        descriptors[i]->featNode.csclcp[2] = descriptors[i]->featNode.csclcp[2]/_scale;

        descriptors[i]->featNode.sum_eigen = descriptors[i]->featNode.sum_eigen/_scale;
        descriptors[i]->featNode.planarity = descriptors[i]->featNode.planarity/_scale;
        descriptors[i]->featNode.anisotropy = descriptors[i]->featNode.anisotropy/_scale;
        descriptors[i]->featNode.sphericity = descriptors[i]->featNode.sphericity/_scale;
        descriptors[i]->featNode.linearity = descriptors[i]->featNode.linearity/_scale;
        descriptors[i]->featNode.omnivariance = descriptors[i]->featNode.omnivariance/_scale;
        descriptors[i]->featNode.eigenentropy = descriptors[i]->featNode.eigenentropy/_scale;
    }

    std::cout<<"DONE GENERATING"<<std::endl;

    return descriptors;
}

void DiffusedNormalVotingClassifier::Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    float radius = ::strtof(_config->GetValue("radius").c_str(), &pEnd);

    for (int i = 0; i < getCloud()->points.size(); i++)
    {
        TensorType T = tensors[i];
        glyphVars glyph = EigenDecomposition(T);
        ComputeSaliencyVals(glyph);
        GlyphAnalysis(glyph);

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

        pointDescriptors[i]->featNode.csclcp[0] += glyph.csclcp[0];
        pointDescriptors[i]->featNode.csclcp[1] += glyph.csclcp[1];
        pointDescriptors[i]->featNode.csclcp[2] += glyph.csclcp[2];

        pointDescriptors[i]->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
        pointDescriptors[i]->featNode.evecs[0] = glyph.evecs[0];
        pointDescriptors[i]->featNode.evecs[1] = glyph.evecs[1];
        pointDescriptors[i]->featNode.evecs[2] = glyph.evecs[2];

        if (radius == _rmin)
        {
            pointDescriptors[i]->featNode.evecs[3] = glyph.evecs[6];
            pointDescriptors[i]->featNode.evecs[4] = glyph.evecs[7];
            pointDescriptors[i]->featNode.evecs[5] = glyph.evecs[8];
        }
        else if (radius == _rmax)
        {
            pointDescriptors[i]->featNode.evecs[6] = glyph.evecs[6];
            pointDescriptors[i]->featNode.evecs[7] = glyph.evecs[7];
            pointDescriptors[i]->featNode.evecs[8] = glyph.evecs[8];
        }

        if(glyph.evals[0] != 0)
        {
            float len = glyph.evals[1] + glyph.evals[0];
            float lamda0 = glyph.evals[0] / len;
            float lamda1 = glyph.evals[1] / len;
            float lamda2 = glyph.evals[2] / len;

            pointDescriptors[i]->featNode.planarity  += (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
            pointDescriptors[i]->featNode.anisotropy += (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
            pointDescriptors[i]->featNode.sphericity += (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];

            pointDescriptors[i]->featNode.linearity += (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
            pointDescriptors[i]->featNode.omnivariance += cbrt(glyph.evals[1] * glyph.evals[2] * glyph.evals[0]);
            pointDescriptors[i]->featNode.eigenentropy += -1*(glyph.evals[0]*log(glyph.evals[0]) + glyph.evals[1]*log(glyph.evals[1]) + glyph.evals[2]*log(glyph.evals[2]));

            pointDescriptors[i]->featNode.eigenvalues[0] = lamda0;
            pointDescriptors[i]->featNode.eigenvalues[1] = lamda1;
            pointDescriptors[i]->featNode.eigenvalues[2] = lamda2;
        }
        else
        {
            pointDescriptors[i]->featNode.planarity = 0;
            pointDescriptors[i]->featNode.anisotropy = 0;
            pointDescriptors[i]->featNode.sphericity = 0;
            pointDescriptors[i]->featNode.linearity = 0;
            pointDescriptors[i]->featNode.omnivariance = 0;
            pointDescriptors[i]->featNode.eigenentropy = 0;
        }
    }
}

void DiffusedNormalVotingClassifier::GetTensor(float radius, std::vector<TensorType>& tensors)
{
    float weight;
    Eigen::Matrix<double, 3, 1> V;
    char* pEnd;
    float lambdaN = ::strtof(_config->GetValue("lambdaN").c_str(), &pEnd);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = getCloud();

    for(int index = 0 ; index < cloud->points.size(); index++)
    {
        for(int j =0; j < 3; j++)
        {
            tensors[index].evec0[j] = 0;
            tensors[index].evec1[j] = 0;
            tensors[index].evec2[j] = 0;
        }

        if (isnan(cloud->points[index].x) || isnan(cloud->points[index].y) || isnan(cloud->points[index].z))
        {
            std::cout<<"The Point at : "<<index<<" NAN : "<<std::endl;
            continue;
        }

        _searchNeighbour->searchOption.searchParameter.radius = radius;
        _neighbourCloud = _searchNeighbour->GetNeighbourCloud(cloud->points[index]);

        for (size_t i = 0; i < _neighbourCloud->points.size() ; i++)
        {
            if (MakeVector(getCloud()->points[index], _neighbourCloud->points[i], &V))
            {
                TensorType ballTensor = Compute3DBallVote(V, &weight);

                for(int j =0; j < 3; j++)
                {
                    tensors[index].evec0[j] = tensors[index].evec0[j] + lambdaN * ballTensor.evec0[j];
                    tensors[index].evec1[j] = tensors[index].evec1[j] + lambdaN * ballTensor.evec1[j];
                    tensors[index].evec2[j] = tensors[index].evec2[j] + lambdaN * ballTensor.evec2[j];
                }
            }
        }

        // Normalize the tensor
        for(int j =0; j < 3; j++)
        {
            tensors[index].evec0[j] = tensors[index].evec0[j] / weight;
            tensors[index].evec1[j] = tensors[index].evec1[j] / weight;
            tensors[index].evec2[j] = tensors[index].evec2[j] / weight;
        }
    }
}

bool DiffusedNormalVotingClassifier::MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V)
{
    Eigen::Matrix<double, 3, 1> temp;
    temp(0,0) = double((neighbour.x - source.x));
    temp(1,0) = double((neighbour.y - source.y));
    temp(2,0) = double((neighbour.z - source.z));

    double len = sqrt(
                temp(0, 0) * temp(0, 0) +
                temp(1, 0) * temp(1, 0) +
                temp(2, 0) * temp(2, 0)
     );

    if(len == 0.0)
        return false;

    (*V)(0, 0) = temp(0, 0);
    (*V)(1, 0) = temp(1, 0);
    (*V)(2, 0) = temp(2, 0);

    return true;
}

TensorType DiffusedNormalVotingClassifier::Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight)
{
    double norm, coeff, s, t;
    Eigen::Matrix3d vv(3,3), voteTemp(3,3);
    TensorType ballTensorVote;
    Eigen::Matrix3d I(3, 3);
    char* pEnd;
    float _sigma = ::strtof(_config->GetValue("sigma").c_str(), &pEnd);

    voteTemp.setZero(3, 3);
    vv.setZero(3, 3);
    I.setIdentity();

    s = V.norm(); // Euclidean Norm

    t = (s*s)/(_sigma * _sigma);

    coeff = exp(-1.0 * t);

    *weight += coeff;

    if(V.norm() != 0.0)
       V = V.normalized();

    norm = V.transpose() * V;
    norm = abs(norm);

    if(norm != 0.0)
    {
        vv = V * V.transpose();
        vv = vv / norm;
    }

    voteTemp = coeff * (I - vv);

    ballTensorVote.evec0[0] = voteTemp(0, 0);
    ballTensorVote.evec0[1] = voteTemp(0, 1);
    ballTensorVote.evec0[2] = voteTemp(0, 2);

    ballTensorVote.evec1[0] = voteTemp(1, 0);
    ballTensorVote.evec1[1] = voteTemp(1, 1);
    ballTensorVote.evec1[2] = voteTemp(1, 2);

    ballTensorVote.evec2[0] = voteTemp(2, 0);
    ballTensorVote.evec2[1] = voteTemp(2, 1);
    ballTensorVote.evec2[2] = voteTemp(2, 2);

    return ballTensorVote;
}

glyphVars DiffusedNormalVotingClassifier::EigenDecomposition(TensorType tensor)
{
    glyphVars glyph;

    float A[3][3], V[3][3], d[3];
    Eigen::Vector3f eigen_values;
    metaVelData diffVel;

    for(int i = 0; i < 3; i++)
    {
        A[i][0] = tensor.evec0[i];
        A[i][1] = tensor.evec1[i];
        A[i][2] = tensor.evec2[i];
    }

    eigen_decomposition(A, V, d); //d[2] > d[1] > d[0]

    eigen_values[0] = d[0] ;
    eigen_values[1] = d[1] ;
    eigen_values[2] = d[2] ;

    float len = d[0] + d[1] + d[2];
    // lamda0>lambda1>lambda2
    float lamda0 = d[2] / len;
    float lamda1 = d[1] / len;
    float lamda2 = d[0] / len;
    Eigen::MatrixXf e0(3,1);
    e0 << V[0][2],V[1][2],V[2][2];
    e0.normalize();
    Eigen::MatrixXf e1(3,1);
    e1 << V[0][1],V[1][1],V[2][1];
    e1.normalize();
    Eigen::MatrixXf e2(3,1);
    e2 << V[0][0],V[1][0],V[2][0];
    e2.normalize();
    Eigen::Matrix3f T;
    T << 0,0,0,0,0,0,0,0,0;

    T += lamda0 * e0 * e0.transpose();
    T += lamda1 * e1 * e1.transpose();
    T += lamda2 * e2 * e2.transpose();

    Getdiffusionvelocity(eigen_values, &diffVel);

    glyph.evals[2] = diffVel.vel[0];       //evals0>evals>1>evals2  //vel2>vel1>vel0
    glyph.evals[1] = diffVel.vel[1];
    glyph.evals[0] = diffVel.vel[2];

    glyph.evecs[0] = V[0][diffVel.index[2]];
    glyph.evecs[1] = V[1][diffVel.index[2]] ;
    glyph.evecs[2] = V[2][diffVel.index[2]];

    glyph.evecs[3] = V[0][diffVel.index[1]];
    glyph.evecs[4] = V[1][diffVel.index[1]] ;
    glyph.evecs[5] = V[2][diffVel.index[1]];

    glyph.evecs[6] = V[0][diffVel.index[0]];
    glyph.evecs[7] = V[1][diffVel.index[0]] ;
    glyph.evecs[8] = V[2][diffVel.index[0]];

    return glyph;
}

void DiffusedNormalVotingClassifier::ComputeSaliencyVals(glyphVars& glyph)
{
    float len = glyph.evals[2] + glyph.evals[1] + glyph.evals[0];

    float cl = 0.0, cp = 0.0, cs = 0.0;

    if(len!= 0.0)
    {
        cl = (glyph.evals[0] - glyph.evals[1])/len; //ev0>ev1>ev2
        cp = (2*(glyph.evals[1] - glyph.evals[2]))/len ;//(2.0 * (eigen_values[1] - eigen_values[0]));
        cs = 1 - (cl+cp); //1.0 - cl - cp;
    }

    glyph.csclcp[0] = cs;
    glyph.csclcp[1] = cl;
    glyph.csclcp[2] = cp;
}

void DiffusedNormalVotingClassifier::GlyphAnalysis(glyphVars& glyph)
{
    double eps=1e-4;
    double evals[3], uv[2], abc[3];

    double norm = sqrt(glyph.evals[2] * glyph.evals[2] + glyph.evals[1]*glyph.evals[1] + glyph.evals[0] *glyph.evals[0]);

    evals[0] = glyph.evals[2];   //ev0>ev1>ev2    //evals0>evals>1>evals2
    evals[1] = glyph.evals[1];
    evals[2] = glyph.evals[0];

    //tenGlyphBqdUvEval(uv, evals);
    //tenGlyphBqdAbcUv(abc, uv, 0.0);  // 3.0 for superquadric glyph, 0.0 for ellpsoid

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

void DiffusedNormalVotingClassifier::Getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel)
{
    char* pEnd;
    float _delta = ::strtof(_config->GetValue("delta").c_str(), &pEnd);

    evals[0] = - evals[0]/_delta;   //ev2>ev1>ev0
    evals[1] = - evals[1]/_delta;
    evals[2] = - evals[2]/_delta;

    evals[0] = exp(evals[0]);
    evals[1] = exp(evals[1]);
    evals[2] = exp(evals[2]);


    (*diffVel).vel[0] = evals[2];    //ev0>ev1>ev2 ->vel2>vel1>vel0
    (*diffVel).vel[1] = evals[1];
    (*diffVel).vel[2] = evals[0];

    (*diffVel).index[0] = 2;
    (*diffVel).index[1] = 1;
    (*diffVel).index[2] = 0;

    return;
}
