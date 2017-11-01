#include "ClassifiersFactory.h"
#include "Classifier.h"

ClassifiersBase* ClassifiersFactory::_instance;

ClassifiersFactory::ClassifiersFactory()
{

}

ClassifiersBase* ClassifiersFactory::GetClassifier()
{
    if (ClassifiersFactory::_instance == 0)
        ClassifiersFactory::_instance = new Classifier();

    return ClassifiersFactory::_instance;
}
