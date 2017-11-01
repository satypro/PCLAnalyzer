#ifndef CLASSIFIERSFACTORY_H
#define CLASSIFIERSFACTORY_H
#include "ClassifiersBase.h"

class ClassifiersFactory
{
public:
    ClassifiersFactory();
    static ClassifiersBase* GetClassifier();
private:
    static ClassifiersBase* _instance;
};

#endif // CLASSIFIERSFACTORY_H
