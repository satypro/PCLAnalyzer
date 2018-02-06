#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H
#include <string>
#include <map>
#include "Views/IViews.h"

class MainController
{
public:
    MainController();
    IViews* Process(std::string controllerName, std::string viewName, std::map<std::string, std::string> request);
};

#endif // MAINCONTROLLER_H
