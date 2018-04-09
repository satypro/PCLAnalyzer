#include "BoundaryTensorClassifier.h"
#include "Descriptors/PointDescriptor.h"
#include "Utilities/eig3.h"
#include <stdlib.h>
#include <teem/ten.h>

BoundaryTensorClassifier::BoundaryTensorClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* BoundaryTensorClassifier::GetConfig()
{
    return _config;
}

std::vector<IPointDescriptor*> BoundaryTensorClassifier::Classify()
{
    size_t cloudSize = getCloud()->points.size();
    std::vector<IPointDescriptor*> descriptors(cloudSize, new PointDescriptor());
    _searchNeighbour = GetSearchStrategy();

    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    // float radius = ::strtof(_config->GetValue("radius").c_str(), &pEnd);
    float _scale = ::strtof(_config->GetValue("scale").c_str(), &pEnd);

    /*if(_scale == 0.0 || _rmin == 0.0 || _rmax == 0.0 || _rmax >= _rmin || getCloud()->points.size() == 0)
    {
        std::cout<<"invalid configuration parameters for classification module"<<std::endl;
        return descriptors;
    }*/

    float dDeltaRadius = (_rmax - _rmin)/(_scale - 1.0);
    float radius = _rmin;


   // while(radius <= _rmax)
    //{
        std::vector<TensorType> boundaryTensors(cloudSize, TensorType());
        std::vector<TensorType> averaged_tensors(cloudSize, TensorType());
        std::cout<<"Progress : Tensor building"<<std::endl;
        BuildBoundaryTensor(radius, boundaryTensors);

        for(size_t i = 0 ; i < cloudSize; ++i)
        {
            MeasureProbability((PointDescriptor*)descriptors[i], averaged_tensors[i], boundaryTensors[i]);
            std::cout<<"Progress : "<<ceil(((float)i/(float)cloudSize)*100)<<"%"<<std::endl;
        }

        radius += dDeltaRadius;
    //}

    return descriptors;
}

void BoundaryTensorClassifier::BuildBoundaryTensor(float radius, std::vector<TensorType>& boundaryTensors)
{
    std::vector<Derivatives> derivatives(getCloud()->points.size(), Derivatives());
    std::cout<<"Progress : Tensor building CalculatePartialDerivative"<<std::endl;
    CalculatePartialDerivative(radius, derivatives);
    std::cout<<"Progress : Tensor building CalculateSecondDerivative"<<std::endl;
    CalculateSecondDerivative(radius, derivatives);
    std::cout<<"Progress : Tensor building CalculateThirdDerivative"<<std::endl;
    CalculateThirdDerivative(radius, derivatives);

    size_t cloudSize = getCloud()->points.size();

    for(size_t i =0; i < cloudSize; i++)
    {
        float Hf[2][2];
        float df[2];
        float Tf[2];

        float G_even[2][2];
        float G_odd[2][2];

        Hf[0][0] = derivatives[i].Ixx;
        Hf[0][1] = derivatives[i].Ixy;
        Hf[1][0] = derivatives[i].Iyx;
        Hf[1][1] = derivatives[i].Iyy;

        df[0] = derivatives[i].Ix;
        df[1] = derivatives[i].Iy;

        Tf[0] = derivatives[i].Ixxx + derivatives[i].Ixyy;
        Tf[1] = derivatives[i].Ixxy + derivatives[i].Iyyy;

        G_even[0][0] = Hf[0][0]*Hf[0][0] + Hf[0][1]*Hf[1][0];
        G_even[0][1] = Hf[0][0]*Hf[0][1] + Hf[0][1]*Hf[1][1];
        G_even[1][0] = Hf[0][0]*Hf[1][0] + Hf[1][1]*Hf[1][0];
        G_even[1][1] = Hf[1][0]*Hf[0][1] + Hf[1][1]*Hf[1][1];

        G_odd[0][0] = -1*df[0]*Tf[0];
        G_odd[0][1] = -0.5*(df[0]*Tf[1]+df[1]*Tf[0]);
        G_odd[1][0] = -0.5*(df[1]*Tf[0]+df[0]*Tf[1]);
        G_odd[1][1] = -1*df[1]*Tf[1];

        boundaryTensors[i].evec0[0] += G_even[0][0] + G_odd[0][0];
        boundaryTensors[i].evec0[1] += G_even[0][1] + G_odd[0][1];

        boundaryTensors[i].evec1[0] += G_even[1][0] + G_odd[1][0];
        boundaryTensors[i].evec1[1] += G_even[1][1] + G_odd[1][1];

        float A[3][3], V[3][3], d[3];

        for(int j = 0; j < 2; j++)
        {
            A[j][0] = boundaryTensors[i].evec0[j];
            A[j][1] = boundaryTensors[i].evec1[j];
            A[j][2] = 0;
        }
        A[2][0] = 0; A[2][1] = 0; A[2][2] = 0;

        eigen_decomposition(A, V, d);

        Eigen::MatrixXf e0(3,1);
        e0 << V[0][0], V[1][0], V[2][0];
        e0.normalize();
        Eigen::MatrixXf e1(3,1);
        e1 << V[0][1], V[1][1], V[2][1];
        e1.normalize();
        Eigen::MatrixXf e2(3,1);
        e2 << V[0][2], V[1][2], V[2][2];
        e2.normalize();
        Eigen::Matrix3f T;
        T << 0,0,0,0,0,0,0,0,0;

        if(d[0]==0)
        {
            T += d[1] * e1 * e1.transpose();
            T += d[2] * e2 * e2.transpose();
            boundaryTensors[i].evec0[0] = T(0,0);
            boundaryTensors[i].evec0[1] = T(0,1);

            boundaryTensors[i].evec1[0] = T(1,0);
            boundaryTensors[i].evec1[1] = T(1,1);
        }

        if(d[1]==0)
        {
            T += d[2] * e2 * e2.transpose();
            boundaryTensors[i].evec0[0] = T(0,0);
            boundaryTensors[i].evec0[1] = T(0,1);

            boundaryTensors[i].evec1[0] = T(1,0);
            boundaryTensors[i].evec1[1] = T(1,1);
        }
    }
}

void BoundaryTensorClassifier::CalculatePartialDerivative(float radius, std::vector<Derivatives>& derivatives)
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

void BoundaryTensorClassifier::CalculateSecondDerivative(float radius, std::vector<Derivatives>& derivatives)
{
    int index = 0;
    _searchNeighbour->searchOption.searchParameter.radius = radius;

    for(pcl::PointXYZ searchPoint : getCloud())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud =
                _searchNeighbour->GetNeighbourCloud(searchPoint);

        float fx0 = 0.0;
        float fy0 = 0.0;
        float fx1 = 0.0;
        float fy1 = 0.0;
        float Wx = 0.0;
        float Wy = 0.0;
        float rxx = 0.0;
        float rxy = 0.0;
        float ryy = 0.0;

        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            rxx += double(neighbourPoint.x - searchPoint.x) * double(neighbourPoint.x - searchPoint.x);
            rxy += double(searchPoint.x - neighbourPoint.x) * double(searchPoint.y - neighbourPoint.y);
            ryy += double(neighbourPoint.y - searchPoint.y) * double(neighbourPoint.y - searchPoint.y);
        }

        float D = rxx*ryy - rxy*rxy;

        int idx = 0;
        std::vector<int> neighbourIndex =  _searchNeighbour->GetNeighbourCloudIndex();
        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            Wx = double(neighbourPoint.x - searchPoint.x) * ryy - double(neighbourPoint.y - searchPoint.y) * rxy;
            Wy = double(neighbourPoint.y - searchPoint.y) * rxx - double(neighbourPoint.x - searchPoint.x) * rxy;
            if(D!=0){
                Wx /= D;
                Wy /= D;
            }
            else{
                Wx = 0;
                Wy = 0;
            }
            fx0 += Wx * double(derivatives[neighbourIndex[idx]].Ix - derivatives[index].Ix);
            fy0 += Wy * double(derivatives[neighbourIndex[idx]].Ix - derivatives[index].Ix);

            fx1 += Wx * double(derivatives[neighbourIndex[idx]].Iy - derivatives[index].Iy);
            fy1 += Wy * double(derivatives[neighbourIndex[idx]].Iy - derivatives[index].Iy);
        }

        derivatives[index].Ixx = fx0;
        derivatives[index].Ixy = fy0;
        derivatives[index].Iyx = fx1;
        derivatives[index].Iyy = fy1;
    }
}

void BoundaryTensorClassifier::CalculateThirdDerivative(float radius, std::vector<Derivatives>& derivatives)
{
    int index = 0;
    _searchNeighbour->searchOption.searchParameter.radius = radius;

    for(pcl::PointXYZ searchPoint : getCloud())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud =
                _searchNeighbour->GetNeighbourCloud(searchPoint);

        float fx0 = 0.0;
        float fy0 = 0.0;
        float fx1 = 0.0;
        float fy1 = 0.0;
        float fx2 = 0.0;
        float fy2 = 0.0;
        float Wx = 0.0;
        float Wy = 0.0;
        float rxx = 0.0;
        float rxy = 0.0;
        float ryy = 0.0;

        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            rxx += double(neighbourPoint.x - searchPoint.x) * double(neighbourPoint.x - searchPoint.x);
            rxy += double(searchPoint.x - neighbourPoint.x) * double(searchPoint.y - neighbourPoint.y);
            ryy += double(neighbourPoint.y - searchPoint.y) * double(neighbourPoint.y - searchPoint.y);
        }

        float D = rxx*ryy - rxy*rxy;

        int idx = 0;
        std::vector<int> neighbourIndex =  _searchNeighbour->GetNeighbourCloudIndex();
        for (pcl::PointXYZ neighbourPoint : _neighbourCloud->points)
        {
            Wx = double(neighbourPoint.x - searchPoint.x) * ryy - double(neighbourPoint.y - searchPoint.y) * rxy;
            Wy = double(neighbourPoint.y - searchPoint.y) * rxx - double(neighbourPoint.x - searchPoint.x) * rxy;
            if(D!=0){
                Wx /= D;
                Wy /= D;
            }
            else{
                Wx = 0;
                Wy = 0;
            }

            fx0 += Wx * double(derivatives[neighbourIndex[idx]].Ixx - derivatives[index].Ixx);
            fy0 += Wy * double(derivatives[neighbourIndex[idx]].Ixx - derivatives[index].Ixx);

            fx1 += Wx * double(derivatives[neighbourIndex[idx]].Ixy - derivatives[index].Ixy);
            fy1 += Wy * double(derivatives[neighbourIndex[idx]].Ixy - derivatives[index].Ixy);

            fx2 += Wx * double(derivatives[neighbourIndex[idx]].Iyy - derivatives[index].Iyy);
            fy2 += Wy * double(derivatives[neighbourIndex[idx]].Iyy - derivatives[index].Iyy);
        }

        derivatives[index].Ixxx = fx0;
        derivatives[index].Ixxy = fy0;
        derivatives[index].Ixyy = fy1;
        derivatives[index].Iyyy = fy2;
    }
}

glyphVars BoundaryTensorClassifier::EigenDecomposition(TensorType boundaryTensor)
{
    float A[3][3], V[3][3], d[3];
    glyphVars glyph;

    for(int i = 0; i < 3; i++)
    {
        A[i][0] = boundaryTensor.evec0[i];
        A[i][1] = boundaryTensor.evec1[i];
        A[i][2] = boundaryTensor.evec2[i];
    }

    eigen_decomposition(A, V, d);

    glyph.evals[2] = d[0];  //d[2] > d[1] > d[0]
    glyph.evals[1] = d[1];
    glyph.evals[0] = d[2];

    glyph.evecs[0] = V[0][2];
    glyph.evecs[1] = V[1][2];
    glyph.evecs[2] = V[2][2];

    glyph.evecs[3] = V[0][1];
    glyph.evecs[4] = V[1][1];
    glyph.evecs[5] = V[2][1];

    glyph.evecs[6] = V[0][0];
    glyph.evecs[7] = V[1][0];
    glyph.evecs[8] = V[2][0];

    return glyph;
}

void BoundaryTensorClassifier::ComputeSaliencyVals(glyphVars& glyph, TensorType& averaged_tensor)
{
    float len = glyph.evals[2] + glyph.evals[1] + glyph.evals[0];

        float cl = 0.0, cp = 0.0, cs = 0.0;

        if(len!= 0.0)
        {
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

            if(lamda0>0)
                T += lamda0 * e0 * e0.transpose();
            if(lamda1>0)
                T += lamda1 * e1 * e1.transpose();
            if(lamda2>0)
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

            if(lamda0>0){
                if(lamda1<0)
                    lamda1=0;
                cl = (lamda0 - lamda1)/(lamda0+lamda1);
                cp = 1-cl;
            }
        }

        glyph.csclcp[1] = cl;
        glyph.csclcp[0] = cs;
        glyph.csclcp[2] = cp;
}

void BoundaryTensorClassifier::GlyphAnalysis(glyphVars& glyph)
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

void BoundaryTensorClassifier::MeasureProbability(PointDescriptor* pointDescriptor, TensorType& averaged_tensor, TensorType& boundaryTensor)
{
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);

    glyphVars glyph = EigenDecomposition(boundaryTensor);
    ComputeSaliencyVals(glyph, averaged_tensor);
    GlyphAnalysis(glyph);

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
    if(glyph.evals[0] != 0)
    {
        float len = glyph.evals[1] + glyph.evals[0];
        float lamda0 = glyph.evals[0] / len;
        float lamda1 = glyph.evals[1] / len;

        pointDescriptor->featNode.planarity = 0;
        pointDescriptor->featNode.anisotropy += (lamda0 -lamda1) / lamda0;
        pointDescriptor->featNode.sphericity += lamda1 / lamda0;
        pointDescriptor->featNode.eigenvalues[0] = lamda0;
        pointDescriptor->featNode.eigenvalues[1] = lamda1;
    }
    else
    {
        pointDescriptor->featNode.planarity = 0;
        pointDescriptor->featNode.anisotropy += 0;
        pointDescriptor->featNode.sphericity += 0;
    }
}
