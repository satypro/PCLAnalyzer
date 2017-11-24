#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H
#include <string>
#include <map>

class MainController
{
public:
    MainController();
    void Process(std::string controllerName, std::string viewName, std::map<std::string, std::string> request);
};

#endif // MAINCONTROLLER_H
