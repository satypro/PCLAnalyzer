#include "ModCovarianceMatrixClassifier.h"
#include "Descriptors/PointDescriptor.h"

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

IPointDescriptor* ModCovarianceMatrixClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;

    return pointDescriptor;
}
