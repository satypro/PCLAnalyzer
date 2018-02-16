#ifndef CLASSIFIERSBASE_H
#define CLASSIFIERSBASE_H
#include <vector>
#include "ClassifierLabels.h"
#include "Descriptors/DescriptorBase.h"
#include "Config/Configuration.h"

class ClassifiersBase
{
public:
    ClassifiersBase();
    virtual IPointDescriptor* Classify(DescriptorBase* descriptor) = 0 ;
    virtual Configuration* GetConfig() = 0;
};

#endif // CLASSIFIERSBASE_H
