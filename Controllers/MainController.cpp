#include "MainController.h"
#include <map>
#include <string>
#include "Models/IViewModel.h"
#include "IControllerBase.h"
#include "Views/IViews.h"
#include "Views/ViewFactory.h"
#include "ControllerFactory.h"

MainController::MainController()
{
}

IViews* MainController::Process(std::string controllerName, std::string viewName, std::map<std::string, std::string> request)
{
    IControllerBase* controller = ControllerFactory::GetController(controllerName);
    IViewModel* model = controller->Process(request);

    IViews* view = ViewFactory::GetView(viewName);
    view->SetViewModel(model);

    return view;
}
