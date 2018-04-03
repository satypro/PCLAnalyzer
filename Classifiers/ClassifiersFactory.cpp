#include "ClassifiersFactory.h"
#include "Classifier.h"

std::map<ClassifierTypes, ClassifiersBase*> ClassifiersFactory::_classifierInstanceMap;

ClassifiersFactory::ClassifiersFactory(){}

ClassifiersBase* ClassifiersFactory::GetInstance(ClassifierTypes classifierType)
{
    std::map<ClassifierTypes, ClassifiersBase*>::iterator it;
    it = ClassifiersFactory::_classifierInstanceMap.find(classifierType);

    if (it != ClassifiersFactory::_classifierInstanceMap.end())
        return it->second;

    ClassifiersBase* instance = ClassifiersFactory::GetClassifier(classifierType);
    ClassifiersFactory::_classifierInstanceMap.insert(
                std::pair<ClassifierTypes, ClassifiersBase*>(classifierType, instance)
     );

    return instance;
}

ClassifiersBase* ClassifiersFactory::GetClassifier(ClassifierTypes classifierType)
{
    ClassifiersBase* object;
    switch(classifierType)
    {
        case C_3DVTGET:
            object = new Classifier();
        break;
        case C_3DVT:
            object = new Classifier();
        break;
        case C_3DCM:
            object = new Classifier();
        break;
        case C_3DMCM:
            object = new Classifier();
        break;
        case C_2DGET:
            object = new Classifier();
        break;
        case C_Hessian:
            object = new Classifier();
        break;
        case C_2DCM:
            object = new Classifier();
        break;
        default:
            object = new Classifier();
    }

    return object;
}
