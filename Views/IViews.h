#ifndef IVIEWS_H
#define IVIEWS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Models/IViewModel.h"

class IViews
{
public:
    virtual void Show() = 0;
    virtual void SetViewModel(IViewModel* model) = 0;
};
#endif // IVIEWS_H
