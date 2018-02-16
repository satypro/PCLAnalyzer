#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include "ClassifiersBase.h"

class Classifier : public ClassifiersBase
{
public:
    Classifier();
    IPointDescriptor* Classify(DescriptorBase* descriptor);
    Configuration* GetConfig();
private:
    Configuration* _config;
};

#endif // CLASSIFIER_H
