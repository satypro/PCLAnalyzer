#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include "ClassifiersBase.h"

class Classifier : public ClassifiersBase
{
public:
    Classifier();
    PointDescriptor Classify(DescriptorBase* descriptor);
    Configuration* GetConfig();
private:
    Configuration* _config;
};

#endif // CLASSIFIER_H
