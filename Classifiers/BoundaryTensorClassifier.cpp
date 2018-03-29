#include "BoundaryTensorClassifier.h"
#include "Descriptors/PointDescriptor.h"

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

IPointDescriptor* BoundaryTensorClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;

    return pointDescriptor;
}
