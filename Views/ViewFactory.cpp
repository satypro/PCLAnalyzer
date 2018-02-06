#include "ViewFactory.h"
#include "IViews.h"
#include "PCLView.h"

ViewFactory::ViewFactory()
{
}

IViews* ViewFactory::GetView(std::string viewName)
{
    IViews* view = 0;
    if (viewName.compare("PCLVIEWER") == 0)
        view = new PCLView();

    return view;
}
