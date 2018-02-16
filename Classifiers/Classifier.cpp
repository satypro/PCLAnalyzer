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

IPointDescriptor* Classifier::Classify(DescriptorBase* descriptor)
{
    return descriptor->GeneratePointDescriptor();
}
