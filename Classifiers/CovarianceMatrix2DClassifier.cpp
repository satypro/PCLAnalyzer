#include "CovarianceMatrix2DClassifier.h"
#include "Descriptors/PointDescriptor.h"

CovarianceMatrix2DClassifier::CovarianceMatrix2DClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* CovarianceMatrix2DClassifier::GetConfig()
{
    return _config;
}

IPointDescriptor* CovarianceMatrix2DClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;

    return pointDescriptor;
}
