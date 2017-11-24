#include "Classifier.h"
#include <vector>
#include "Utilities/CommonUtility.h"
#include "Utilities/eig3.h"

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

std::vector<ClassLabels> Classifier::Classify(DescriptorBase* descriptor)
{
    std::vector<ClassLabels> labels;

    //1. Get the Covanariance Matrix and then get the Eigen Values
    //2. Example line  e0~e1>>e2
    TensorType T = descriptor->Get3DVotingTensor();

    float A[3][3], V[3][3], d[3];
    //Eigen::Vector3f eigen_values;
    //Eigen::Matrix3f V;

    /*
    V(0, 0 ) = T.evec0[0];
    V(0, 1 ) = T.evec0[1];
    V(0, 2 ) = T.evec0[2];

    V(1, 0 ) = T.evec1[0];
    V(1, 1 ) = T.evec1[1];
    V(1, 2 ) = T.evec1[2];

    V(2, 0 ) = T.evec2[0];
    V(2, 1 ) = T.evec2[1];
    V(2, 2 ) = T.evec2[2];
    */

    A[0][0] = T.evec0[0];
    A[0][1] = T.evec0[1];
    A[0][2] = T.evec0[2];

    A[1][0] = T.evec1[0];
    A[1][1] = T.evec1[1];
    A[1][2] = T.evec1[2];

    A[2][0] = T.evec2[0];
    A[2][1] = T.evec2[1];
    A[2][2] = T.evec2[2];


    std::cout<<"Eigen vector Before : "<<std::endl;
    //std::cout<<"Eigen 0 : " << T.evec0[0] <<" "<< T.evec0[1] <<" "<< T.evec0[2]<< std::endl;
    //std::cout<<"Eigen 1 : " << T.evec1[0] <<" "<< T.evec1[1] <<" "<< T.evec1[2]<< std::endl;
    //std::cout<<"Eigen 2 : " << T.evec2[0] <<" "<< T.evec2[1] <<" "<< T.evec2[2]<< std::endl;

    // To Get the EigenValue and EigenVector
    EigenResult result;// = CommonUtility::ComputeEigen(V);
    eigen_decomposition(A, V, d);

    result.EigenValues(0,0) = d[0];
    result.EigenValues(1,0) = d[1];
    result.EigenValues(2,0) = d[2];

   /*
    std::cout<<"Eigen Values Before : "<<std::endl;
    std::cout<<"Eigen 0 : " << result.EigenValues(0,0)<< std::endl;
    std::cout<<"Eigen 1 : " << result.EigenValues(1,0)<< std::endl;
    std::cout<<"Eigen 2 : " << result.EigenValues(2,0)<< std::endl;
    */

    float delta = 0.16;

    result.EigenValues(0,0) = (float)exp(-1 * (d[0]/delta));
    result.EigenValues(1,0) = (float)exp(-1 * (d[1]/delta));
    result.EigenValues(2,0) = (float)exp(-1 * (d[2]/delta));


    std::cout<<"Eigen Values After : "<<std::endl;
    std::cout<<"Eigen 0 : " <<d[0]<< std::endl;
    std::cout<<"Eigen 1 : " << result.EigenValues(0,0)<< std::endl;
    std::cout<<"Eigen 2 : " << exp(-1 * (d[0]/delta))<< std::endl;

    if (result.EigenValues(0,0) == 0.0 && result.EigenValues(2,0) == 0.0  && result.EigenValues(1,0) == 0.0 )
    {
        labels.push_back(Point);
        return labels;
    }

    if (0.5 * result.EigenValues(0,0) <= result.EigenValues(2,0))
    {
        labels.push_back(Point);
        return labels;
    }

    if (0.5 * result.EigenValues(0,0) > result.EigenValues(1,0))
    {
        labels.push_back(Curve);
        return labels;
    }

    if (0.5 * result.EigenValues(0,0) > result.EigenValues(2,0))
    {
        labels.push_back(Disc);
        return labels;
    }

    return labels;
}
