#ifndef ICONTROLLER_H
#define ICONTROLLER_H
#include "Models/IViewModel.h"
#include <map>
#include <string>

class IControllerBase
{
public:
    IControllerBase()
    {
    };
    virtual IViewModel* Process(std::map<std::string, std::string> request) = 0;
};

#endif // ICONTROLLER_H
