#include "ControllerFactory.h"
#include "ProcessController.h"

ControllerFactory::ControllerFactory()
{

}

IControllerBase* ControllerFactory::GetController(std::string controllerName)
{
    IControllerBase* controller = 0;
    if (controllerName.compare("PROCESS"))
        controller = new ProcessController();

    return controller;
}
