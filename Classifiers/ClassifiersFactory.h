#ifndef CLASSIFIERSFACTORY_H
#define CLASSIFIERSFACTORY_H
#include <map>
#include "ClassifiersBase.h"
#include "ClassifierType.h"

class ClassifiersFactory
{
public:
    ClassifiersFactory();
    static ClassifiersBase* GetClassifier(ClassifierTypes classifierType);
private:    
    static std::map<ClassifierTypes, ClassifiersBase*> _classifierInstanceMap;
    static ClassifiersBase* GetInstance(ClassifierTypes classifierType);
};

#endif // CLASSIFIERSFACTORY_H
