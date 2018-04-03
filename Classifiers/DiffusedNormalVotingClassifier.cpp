#include "DiffusedNormalVotingClassifier.h"
#include "Descriptors/PointDescriptor.h"

DiffusedNormalVotingClassifier::DiffusedNormalVotingClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* DiffusedNormalVotingClassifier::GetConfig()
{
    return _config;
}

IPointDescriptor* DiffusedNormalVotingClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;

    return pointDescriptor;
}