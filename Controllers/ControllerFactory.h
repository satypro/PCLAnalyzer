#ifndef CONTROLLERFACTORY_H
#define CONTROLLERFACTORY_H
#include "IControllerBase.h"

class ControllerFactory
{
public:
    ControllerFactory();
    static IControllerBase* GetController(std::string controllerName);
};

#endif // CONTROLLERFACTORY_H
