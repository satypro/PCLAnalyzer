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
    std::vector<IPointDescriptor*> descriptors;

    return descriptors;
}
