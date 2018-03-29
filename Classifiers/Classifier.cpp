#include "Classifier.h"
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <stdlib.h>
#include <teem/ten.h>
#include "Utilities/CommonUtility.h"
#include "Utilities/eig3.h"
#include "Descriptors/PointDescriptor.h"

Classifier::Classifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* Classifier::GetConfig()
{
    return _config;
}

IPointDescriptor* Classifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;

    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    float radius = ::strtof(_config->GetValue("radius").c_str(), &pEnd);

    TensorType T = Get3DVotingTensor();
    glyphVars glyph = EigenDecomposition(T);

    computeSaliencyVals(glyph);
    glyphAnalysis(glyph);
    pointDescriptor->glyph = glyph;

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

    pointDescriptor->featNode.csclcp[0] += glyph.csclcp[0];
    pointDescriptor->featNode.csclcp[1] += glyph.csclcp[1];
    pointDescriptor->featNode.csclcp[2] += glyph.csclcp[2];

    pointDescriptor->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
    pointDescriptor->featNode.evecs[0] = glyph.evecs[0];
    pointDescriptor->featNode.evecs[1] = glyph.evecs[1];
    pointDescriptor->featNode.evecs[2] = glyph.evecs[2];

    if (radius == _rmin)
    {
        pointDescriptor->featNode.evecs[3] = glyph.evecs[6];
        pointDescriptor->featNode.evecs[4] = glyph.evecs[7];
        pointDescriptor->featNode.evecs[5] = glyph.evecs[8];
    }
    else if (radius == _rmax)
    {
        pointDescriptor->featNode.evecs[6] = glyph.evecs[6];
        pointDescriptor->featNode.evecs[7] = glyph.evecs[7];
        pointDescriptor->featNode.evecs[8] = glyph.evecs[8];
    }

    if(glyph.evals[0] != 0)
    {
        float len = glyph.evals[1] + glyph.evals[0];
        float lamda0 = glyph.evals[0] / len;
        float lamda1 = glyph.evals[1] / len;
        float lamda2 = glyph.evals[2] / len;

        pointDescriptor->featNode.planarity  += (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
        pointDescriptor->featNode.anisotropy += (glyph.evals[1] - glyph.evals[2]) / glyph.evals[0];
        pointDescriptor->featNode.sphericity += (glyph.evals[0] - glyph.evals[2]) / glyph.evals[0];

        pointDescriptor->featNode.linearity += (glyph.evals[0] - glyph.evals[1]) / glyph.evals[0];
        pointDescriptor->featNode.omnivariance += cbrt(glyph.evals[1] * glyph.evals[2] * glyph.evals[0]);
        pointDescriptor->featNode.eigenentropy += -1*(glyph.evals[0]*log(glyph.evals[0]) + glyph.evals[1]*log(glyph.evals[1]) + glyph.evals[2]*log(glyph.evals[2]));

        pointDescriptor->featNode.eigenvalues[0] = lamda0;
        pointDescriptor->featNode.eigenvalues[1] = lamda1;
        pointDescriptor->featNode.eigenvalues[2] = lamda2;
    }
    else
    {
        pointDescriptor->featNode.planarity = 0;
        pointDescriptor->featNode.anisotropy = 0;
        pointDescriptor->featNode.sphericity = 0;
        pointDescriptor->featNode.linearity = 0;
        pointDescriptor->featNode.omnivariance = 0;
        pointDescriptor->featNode.eigenentropy = 0;
    }

    float A[3][3], V[3][3], d[3];

    A[0][0] = T.evec0[0];
    A[0][1] = T.evec0[1];
    A[0][2] = T.evec0[2];

    A[1][0] = T.evec1[0];
    A[1][1] = T.evec1[1];
    A[1][2] = T.evec1[2];

    A[2][0] = T.evec2[0];
    A[2][1] = T.evec2[1];
    A[2][2] = T.evec2[2];

    // To Get the EigenValue and EigenVector
    EigenResult result;
    eigen_decomposition(A, V, d);

    result.EigenValues(0,0) = d[0];
    result.EigenValues(1,0) = d[1];
    result.EigenValues(2,0) = d[2];

    float delta = 0.16;

    result.EigenValues(0,0) = (float)exp(-1 * (d[0]/delta));
    result.EigenValues(1,0) = (float)exp(-1 * (d[1]/delta));
    result.EigenValues(2,0) = (float)exp(-1 * (d[2]/delta));

    if (result.EigenValues(0,0) == 0.0 && result.EigenValues(2,0) == 0.0  && result.EigenValues(1,0) == 0.0 )
    {
        pointDescriptor->label = Point;
        return pointDescriptor;
    }

    if (0.5 * result.EigenValues(0,0) <= result.EigenValues(2,0))
    {
        pointDescriptor->label = Point;
        return pointDescriptor;
    }

    if (0.5 * result.EigenValues(0,0) > result.EigenValues(1,0))
    {
        pointDescriptor->label = Curve;
        return pointDescriptor;
    }

    if (0.5 * result.EigenValues(0,0) > result.EigenValues(2,0))
    {
        pointDescriptor->label = Disc;
        return pointDescriptor;
    }

    return pointDescriptor;
}

Eigen::Vector4f Classifier::Get3DCentroid()
{
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*getCloud(), xyz_centroid);

    return xyz_centroid;
}

Eigen::Matrix3f Classifier::ComputeCovarianceMatrix()
{
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrix(*getCloud(), Get3DCentroid(), covariance_matrix);

    return covariance_matrix;
}

TensorType Classifier::Get3DVotingTensor()
{
    TensorType tensor;
    TensorType result;
    float weight;
    Eigen::Matrix<double, 3, 1> V;
    char* pEnd;
    float lambdaN = ::strtof(_config->GetValue("lambdaN").c_str(), &pEnd);

    for(int j =0; j < 3; j++)
    {
        result.evec0[j] = 0;
        result.evec1[j] = 0;
        result.evec2[j] = 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = getCloud();

    for (size_t i = 0; i < neighbourCloud->points.size() ; i++)
    {
        if (MakeVector(getSource(), neighbourCloud->points[i], &V))
        {
            tensor = Compute3DBallVote(V, &weight);

            for(int j =0; j < 3; j++)
            {
                result.evec0[j] = result.evec0[j] + lambdaN * tensor.evec0[j];
                result.evec1[j] = result.evec1[j] + lambdaN * tensor.evec1[j];
                result.evec2[j] = result.evec2[j] + lambdaN * tensor.evec2[j];
            }
        }
    }

    // Normalize the tensor
   /* for(int j =0; j < 3; j++)
    {
        result.evec0[j] = result.evec0[j] / weight;
        result.evec1[j] = result.evec1[j] / weight;
        result.evec2[j] = result.evec2[j] / weight;
    }
    */

    return result;
}

TensorType Classifier::GetCoVaraianceTensor()
{
    TensorType tensor;

    Eigen::Matrix3f covariance_matrix = ComputeCovarianceMatrix();

    for (int k = 0 ; k < 3 ; k++)
    {
        for (int j = 0 ; j < 3; j++)
        {
            if (k == 0)
                tensor.evec0[j] = covariance_matrix(k, j);
            if (k == 1)
                tensor.evec1[j] = covariance_matrix(k, j);
            if (k == 2)
                tensor.evec2[j] = covariance_matrix(k, j);
        }
    }

    return tensor;
}

/*Private methods*/
bool Classifier::MakeVector(pcl::PointXYZ source, pcl::PointXYZ neighbour, Eigen::Matrix<double, 3, 1>* V)
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

TensorType Classifier::Compute3DBallVote(Eigen::Matrix<double, 3, 1> V, float *weight)
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

glyphVars Classifier::EigenDecomposition(TensorType tensor)
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

    getdiffusionvelocity(eigen_values, &diffVel);

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

void Classifier::getdiffusionvelocity(Eigen::Vector3f evals, metaVelData *diffVel)
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

void Classifier::computeSaliencyVals(glyphVars& glyph)
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

void Classifier::glyphAnalysis(glyphVars& glyph)
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
/*End the private method region */
