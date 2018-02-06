#ifndef IVIEWS_H
#define IVIEWS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Models/ViewModel.h"

class IViews
{
public:
    virtual void Show() = 0;
    ViewModel model;
};
#endif // IVIEWS_H
