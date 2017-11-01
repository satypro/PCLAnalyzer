#ifndef DESCRIPTORFACTORY_H
#define DESCRIPTORFACTORY_H
#include "DescriptorBase.h"

class DescriptorFactory
{
public:
    DescriptorFactory();
    static DescriptorBase* GetDescriptor();
private:
    static DescriptorBase* _instance;
};

#endif // DESCRIPTORFACTORY_H
