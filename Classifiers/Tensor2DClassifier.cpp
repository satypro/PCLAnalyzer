#include "Tensor2DClassifier.h"
#include "Descriptors/PointDescriptor.h"

Tensor2DClassifier::Tensor2DClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* Tensor2DClassifier::GetConfig()
{
    return _config;
}

std::vector<IPointDescriptor*> Tensor2DClassifier::Classify()
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
        std::vector<TensorType> averaged_tensor(cloudSize, TensorType());
        GetCoVaraianceTensor(radius, tensors);
        Process(descriptors, tensors, averaged_tensor);
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
        descriptor.sphericity /= _scale;
        descriptor.anisotropy /= _scale;
        descriptor.sum_eigen /= _scale;
    }

    return descriptors;
}

void Tensor2DClassifier::Process(std::vector<PointDescriptor*>& pointDescriptors, std::vector<TensorType>& tensors, std::vector<TensorType>& averaged_tensor)
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

        pointDescriptors[i]->featNode.sum_eigen = glyph.evals[0] + glyph.evals[1] + glyph.evals[2];
        if(glyph.evals[0] != 0)
        {
            float len = glyph.evals[1] + glyph.evals[0];
            float lamda0 = glyph.evals[0] / len;
            float lamda1 = glyph.evals[1] / len;

            pointDescriptors[i]->featNode.planarity = 0;
            pointDescriptors[i]->featNode.anisotropy += (lamda0 -lamda1) / lamda0;
            pointDescriptors[i]->featNode.sphericity += lamda1 / lamda0;
        }
        else
        {
            pointDescriptors[i]->featNode.planarity += 0;
            pointDescriptors[i]->featNode.anisotropy += 0;
            pointDescriptors[i]->featNode.sphericity += 0;
        }
    }
}


void Tensor2DClassifier::CalculatePartialDerivative(float radius, std::vector<Derivatives>& derivatives)
{
    int index = 0;
    for(pcl::PointXYZ searchPoint : getCloud())
    {
        _searchNeighbour->searchOption.searchParameter.radius = radius;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud =
                _searchNeighbour->GetNeighbourCloud(searchPoint);

        float Wx = 0.0;
        float Wy = 0.0;

        float rxx = 0.0;
        float rxy = 0.0;
        float ryy = 0.0;

        float fx0 = 0.0;
        float fy0 = 0.0;

        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            rxx += double(neighbourPoint.x - searchPoint.x)
                    * double(neighbourPoint.x - searchPoint.x);

            rxy += double(searchPoint.x - neighbourPoint.x)
                    * double(searchPoint.y - neighbourPoint.y);

            ryy += double(neighbourPoint.y - searchPoint.y)
                    * double(neighbourPoint.y - searchPoint.y);
        }

        float D = rxx * ryy - rxy * rxy;

        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            Wx = double(neighbourPoint.x - searchPoint.x) * ryy - double(neighbourPoint.y - searchPoint.y) * rxy;
            Wy = double(neighbourPoint.y - searchPoint.y) * rxx - double(neighbourPoint.x - searchPoint.x) * rxy;
            if(D>0.00001){
                 Wx /= D;
                Wy /= D;
            }

            fx0 += Wx * double(neighbourPoint.z - searchPoint.z);
            fy0 += Wy * double(neighbourPoint.z - searchPoint.z);
        }

        derivatives[index].Ix = fx0;
        derivatives[index].Iy = fy0;
    }
}

    void Tensor2DClassifier::Tensor2D(float radius, std::vector<TensorType>& tensor2Ds)
    {
        std::vector<Derivatives> derivatives(getCloud()->points.size(), Derivatives());
        CalculatePartialDerivative(radius, derivatives);

        _searchNeighbour->searchOption.searchParameter.radius = radius;

        int index = 0;
        for(pcl::PointXYZ searchPoint : getCloud()->points)
        {   
            float weight = 0.0;

            pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud =
                    _searchNeighbour->GetNeighbourCloud(searchPoint);

            for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
            {
                ColumnVector Vect1;
                Vect1(0,0) = double(neighbourPoint.x - searchPoint.x);
                Vect1(1,0) = double(neighbourPoint.y - searchPoint.y);
                Vect1(2,0) = double(neighbourPoint.z - searchPoint.z);

                double  s = Vect1.norm();

                double coeff = 1/(2*3.141592653*radius*radius)*exp (-1 * s*s/(2*radius*radius));

                weight+= coeff;

                tensor2Ds[i].evec0[0] += coeff * derivatives[i].Ix * derivatives[i].Ix;
                tensor2Ds[i].evec0[1] += coeff * derivatives[i].Ix * derivatives[i].Iy;
                tensor2Ds[i].evec0[2] = 0;

                tensor2Ds[i].evec1[0] += coeff * derivatives[i].Ix * derivatives[i].Iy;
                tensor2Ds[i].evec1[1] += coeff * derivatives[i].Iy * derivatives[i].Iy;
                tensor2Ds[i].evec1[2] = 0;

                tensor2Ds[i].evec2[0] = 0;
                tensor2Ds[i].evec2[1] = 0;
                tensor2Ds[i].evec2[2] = 0;
            }

            if(weight != 0.0)
            {
                for(int j =0; j < 3; j++)
                {
                    tensor2Ds[i].evec0[j] = tensor2Ds[i].evec0[j]/weight;
                    tensor2Ds[i].evec1[j] = tensor2Ds[i].evec1[j]/weight;
                    tensor2Ds[i].evec2[j] = tensor2Ds[i].evec2[j]/weight;
                }
            }   
        }
    }

glyphVars Tensor2DClassifier::EigenDecomposition(TensorType tensor)
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
}

void Tensor2DClassifier::glyphAnalysis(glyphVars& glyph)
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