#ifndef PROCESSCONTROLLER_H
#define PROCESSCONTROLLER_H
#include "IControllerBase.h"
#include <map>
#include <string>

class ProcessController : public IControllerBase
{
public:
    ProcessController();
    IViewModel* Process(std::map<std::string, std::string> request);
};

#endif // PROCESSCONTROLLER_H
