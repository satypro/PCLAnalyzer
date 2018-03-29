#include "CovarianceMatrixClassifier.h"
#include "Descriptors/PointDescriptor.h"

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

    return pointDescriptor;
}
