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
    std::vector<IPointDescriptor*> descriptors;

    return descriptors;
}
