#ifndef PCLVIEW_H
#define PCLVIEW_H
#include "IViews.h"
#include "Models/ViewModel.h"

class PCLView : public IViews
{
public:
    PCLView();
    void Show(ViewModel model);
};

#endif // PCLVIEW_H
