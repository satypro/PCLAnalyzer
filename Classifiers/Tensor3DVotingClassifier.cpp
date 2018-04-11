#include "Tensor3DVotingClassifier.h"
#include "Descriptors/PointDescriptor.h"

Tensor3DVotingClassifier::Tensor3DVotingClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* Tensor3DVotingClassifier::GetConfig()
{
    return _config;
}

std::vector<IPointDescriptor*> Tensor3DVotingClassifier::Classify()
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
        Tensor3DVoting(radius, tensors);
        Process(descriptors, tensors);
        radius += dDeltaRadius;
    }

    for(PointDescriptor* descriptor : descriptors)
    {
        descriptor.prob[0] = descriptor.prob[0]/_scale;
        descriptor.prob[1] = descriptor.prob[1]/_scale;
        descriptor.prob[2] = descriptor.prob[2]/_scale;

        descriptor.featStrength[0] = descriptor.featStrength[0]/_scale;
        descriptor.featStrength[1] = descriptor.featStrength[1]/_scale;
        descriptor.featStrength[2] = descriptor.featStrength[2]/_scale;

        descriptor.csclcp[0] = descriptor.csclcp[0]/_scale;
        descriptor.csclcp[1] = descriptor.csclcp[1]/_scale;
        descriptor.csclcp[2] = descriptor.csclcp[2]/_scale;
    }

    return descriptors;
}

void Tensor3DVotingClassifier::Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);

    for (int i = 0; getCloud()->points.size(); i++)
    {
        TensorType T = tensors[i];
        glyphVars glyph = EigenDecomposition(T);
        computeSaliencyVals(glyph);
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

        pointDescriptors[i]->featNode.csclcp[0] += glyph.csclcp[0];
        pointDescriptors[i]->featNode.csclcp[1] += glyph.csclcp[1];
        pointDescriptors[i]->featNode.csclcp[2] += glyph.csclcp[2];
    }
}

void Tensor3DVotingClassifier::Tensor3DVoting(float radius, std::vector<TensorType>& tensor3DVotins)
{
     _searchNeighbour->searchOption.searchParameter.radius = radius;

    int index = -1;
    float weight = 0.0;
    for(pcl::PointXYZ searchPoint : getCloud()->points)
    {   
        index++;
        weight = 0.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud = _searchNeighbour->GetNeighbourCloud(searchPoint);

        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            Eigen::Matrix<double, 3, 1> Vect1;
            Matrix3d vv(3,3), voteTemp(3,3);

            Vect1(0,0) = double(neighbourPoint.x - searchPoint.x);
            Vect1(1,0) = double(neighbourPoint.y - searchPoint.y);
            Vect1(2,0) = double(neighbourPoint.z - searchPoint.z);

            double  s = Vect1.norm(); // if s is zero

            double coeff = radius - s;
            weight+= coeff;

            vv = Vect1 * Vect1.transpose();

            voteTemp =  coeff * vv;

            tensor3DVotins[i].evec0[0] += voteTemp(0,0);
            tensor3DVotins[i].evec0[1] += voteTemp(0,1);
            tensor3DVotins[i].evec0[2] += voteTemp(0,2);

            tensor3DVotins[i].evec1[0] += voteTemp(1,0);
            tensor3DVotins[i].evec1[1] += voteTemp(1,1);
            tensor3DVotins[i].evec1[2] += voteTemp(1,2);

            tensor3DVotins[i].evec2[0] += voteTemp(2,0);
            tensor3DVotins[i].evec2[1] += voteTemp(2,1);
            tensor3DVotins[i].evec2[2] += voteTemp(2,2);
        }

        if (weight != 0.0 )
        {
            for(int j =0; j < 3; j++)
            {
                tensor3DVotins[i].evec0[j] = tensor3DVotins[i].evec0[j]/weight;
                tensor3DVotins[i].evec1[j] = tensor3DVotins[i].evec1[j]/weight;
                tensor3DVotins[i].evec2[j] = tensor3DVotins[i].evec2[j]/weight;
            }
        }
    }
}

glyphVars Tensor3DVotingClassifier::EigenDecomposition(TensorType tensor)
{
    float A[3][3], V[3][3], d[3];
    glyphVars glyph;

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

    return glyph;
}

void Tensor3DVotingClassifier::ComputeSaliencyVals(glyphVars& glyph)
{
    float len = glyph.evals[2] + glyph.evals[1] + glyph.evals[0];

    float cl = 0.0, cp = 0.0, cs = 0.0;

    if(len!= 0.0)
    {
        cl = (glyph.evals[0] - glyph.evals[1])/len; //ev0>ev1>ev2
        cp = (2*(glyph.evals[1] - glyph.evals[2]))/len ;
        cs = 1 - (cl+cp); //1.0 - cl - cp;
    }

    glyph.csclcp[1] = cl;
    glyph.csclcp[0] = cs;
    glyph.csclcp[2] = cp;
}

void Tensor3DVotingClassifier::glyphAnalysis(glyphVars& glyph)
{
    double eps=1e-4;
    double evals[3], uv[2], abc[3];

    double norm = sqrt(glyph.evals[2] * glyph.evals[2] + glyph.evals[1]*glyph.evals[1] + glyph.evals[0] *glyph.evals[0]);

    if(norm != 0)
    {
        // normalized the eigenvalues for superquadric glyph 
        // such that sqrt(lamda^2 + lambad^1 +lambda^0) = 1
        glyph.evals[2] = glyph.evals[2]/norm;  
        glyph.evals[1] = glyph.evals[1]/norm;
        glyph.evals[0] = glyph.evals[0]/norm;
    }

    evals[0] = glyph.evals[2]; //ev0>ev1>ev2
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