#ifndef CLASSIFIERSBASE_H
#define CLASSIFIERSBASE_H
#include <vector>
#include "ClassifierLabels.h"
#include "Descriptors/DescriptorBase.h"

class ClassifiersBase
{
public:
    ClassifiersBase();
    virtual std::vector<ClassLabels> Classify(DescriptorBase* descriptor) = 0 ;
};

#endif // CLASSIFIERSBASE_H
