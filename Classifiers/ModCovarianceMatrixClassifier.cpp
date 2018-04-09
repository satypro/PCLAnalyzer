#include "ModCovarianceMatrixClassifier.h"
#include "Descriptors/PointDescriptor.h"
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <teem/ten.h>
#include "Utilities/eig3.h"

ModCovarianceMatrixClassifier::ModCovarianceMatrixClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* ModCovarianceMatrixClassifier::GetConfig()
{
    return _config;
}

std::vector<IPointDescriptor*> ModCovarianceMatrixClassifier::Classify()
{
    std::vector<IPointDescriptor*> descriptors;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = getCloud();
    size_t size = cloud->points.size ();
    for (size_t i = 0; i < size; ++i)
    {
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }

        this->setSource(cloud->points[i]);
        IPointDescriptor* pointdescriptor = Process();
        descriptors.push_back(pointdescriptor);
        std::cout<<"Progress : "<<ceil(((float)i/(float)size)*100)<<"%"<<std::endl;
    }

    return descriptors;
}

IPointDescriptor* ModCovarianceMatrixClassifier::Process()
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

    // We swap the averaged_tensor and covarianceTensor
    MeasureProbability(pointDescriptor, covarianceTensor, averaged_tensor);

    return pointDescriptor;
}

TensorType ModCovarianceMatrixClassifier::GetCoVaraianceTensor(float radius)
{
    pcl::PointXYZ pointxyz = getSource();
    ColumnVector Vect1;
    Matrix3d vv(3,3), voteTemp(3,3);
    TensorType covarianceTensor;

    _searchNeighbour->searchOption.searchParameter.radius = radius;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud = _searchNeighbour->GetNeighbourCloud(pointxyz);

    float  weight = 0.0;

    for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
    {
        ColumnVector Vect1;
        Matrix3d vv(3,3), voteTemp(3,3);

        Vect1(0,0) = double((searchPoint.x -  neighbourPoint.x));
        Vect1(1,0) = double((searchPoint.y - neighbourPoint.y));
        Vect1(2,0) = double((searchPoint.z - neighbourPoint.z));

        double  s = Vect1.norm(); // if s is zero

        double t = (s*s)/(_sigma * _sigma);

        double coeff = exp(-1.0 * t);

        weight+= coeff;

        if(Vect1.norm() != 0.0)
        Vect1 = Vect1.normalized();

        double norm = Vect1.transpose() * Vect1;
        norm = abs(norm);

        if(norm != 0.0)
        {
          vv = Vect1 * Vect1.transpose();
          vv = vv / norm;  // if norm is zero
        }

        voteTemp = coeff * vv;

        covarianceTensor.evec0[0] += voteTemp(0,0);
        covarianceTensor.evec0[1] += voteTemp(0,1);
        covarianceTensor.evec0[2] += voteTemp(0,2);

        covarianceTensor.evec1[0] += voteTemp(1,0);
        covarianceTensor.evec1[1] += voteTemp(1,1);
        covarianceTensor.evec1[2] += voteTemp(1,2);

        covarianceTensor.evec2[0] += voteTemp(2,0);
        covarianceTensor.evec2[1] += voteTemp(2,1);
        covarianceTensor.evec2[2] += voteTemp(2,2);
    }

    if(weight != 0.0)
    {
        for(int j =0; j < 3; j++)
        {
                covarianceTensor.evec0[j] = covarianceTensor.evec0[j]/weight;
                covarianceTensor.evec1[j] = covarianceTensor.evec1[j]/weight;
                covarianceTensor.evec2[j] = covarianceTensor.evec2[j]/weight;
        }
    }

    return covarianceTensor;
}

void ModCovarianceMatrixClassifier::MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& covarianceTensor)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);

    glyphVars glyph = EigenDecomposition(covarianceTensor);
    computeSaliencyVals(glyph, averaged_tensor);
    glyphAnalysis(glyph);

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
}

glyphVars ModCovarianceMatrixClassifier::EigenDecomposition(TensorType tensor)
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
    glyph.evecs[1] = V[1][2] ;
    glyph.evecs[2] = V[2][2];

    glyph.evecs[3] = V[0][1];
    glyph.evecs[4] = V[1][1] ;
    glyph.evecs[5] = V[2][1];

    glyph.evecs[6] = V[0][0];
    glyph.evecs[7] = V[1][0] ;
    glyph.evecs[8] = V[2][0];
}

void ModCovarianceMatrixClassifier::computeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor)
{
    float len = glyph.evals[2] + glyph.evals[1] + glyph.evals[0];

    float cl = 0.0, cp = 0.0, cs = 0.0;

    if(len!= 0.0)
    {
        cl = (glyph.evals[0] - glyph.evals[1])/len; //ev0>ev1>ev2
        cp = (2*(glyph.evals[1] - glyph.evals[2]))/len ;//(2.0 * (eigen_values[1] - eigen_values[0]));
        cs = 1 - (cl+cp); //1.0 - cl - cp;
    }

    glyph.csclcp[1] = cl;
    glyph.csclcp[0] = cs;
    glyph.csclcp[2] = cp;
}

void ModCovarianceMatrixClassifier::glyphAnalysis(glyphVars& glyph)
{
    double eps=1e-4;
    double evals[3], uv[2], abc[3];

    double norm = sqrt(glyph.evals[2] * glyph.evals[2] + glyph.evals[1]*glyph.evals[1] + glyph.evals[0] *glyph.evals[0]);

    if(norm != 0)
    {
        glyph.evals[2] = glyph.evals[2]/norm;  //normalized the eigenvalues for superquadric glyph such that sqrt(lamda^2 + lambad^1 +lambda^0) = 1
        glyph.evals[1] = glyph.evals[1]/norm;
        glyph.evals[0] = glyph.evals[0]/norm;

    }

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
