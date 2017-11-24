#ifndef VIEWFACTORY_H
#define VIEWFACTORY_H
#include <string>
#include "IViews.h"

class ViewFactory
{
public:
    ViewFactory();
    static IViews* GetView(std::string viewName);
};

#endif // VIEWFACTORY_H
