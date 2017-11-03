#include "Classifier.h"
#include <vector>
#include "Utilities/CommonUtility.h"

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
    //2. example line  e0~e1>>e2

    Eigen::Matrix3f covariance_matrix = descriptor->ComputeCovarianceMatrix();
    EigenResult result = CommonUtility::ComputeEigen(covariance_matrix);

    if (result.EigenValues(0,0) > result.EigenValues(2,0) && result.EigenValues(1,0) > result.EigenValues(2,0))
    {
        labels.push_back(Disc);
        return labels;
    }

    if (result.EigenValues(0,0) > result.EigenValues(1,0) && result.EigenValues(0,0) > result.EigenValues(2,0))
    {
        labels.push_back(Curve);
        return labels;
    }

    labels.push_back(Point);

    return labels;
}
