#ifndef PROCESSCONTROLLER_H
#define PROCESSCONTROLLER_H
#include "IControllerBase.h"
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include "Classifiers/ClassifiersBase.h"
#include "Neighbours/SearchNeighbourBase.h"
#include "Models/ViewModel.h"

class ProcessController : public IControllerBase
{
public:
    ProcessController();
    IViewModel* Process(std::map<std::string, std::string> request);
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadPointCloud(std::string filePath);
    ClassifiersBase* GetClassifier(std::string classifierSelected, std::map<std::string, std::string> request);
    SearchNeighbourBase* GetNeighbourSearchStrategy(SearchOption option);
    void StructFeatClassification(ViewModel* model);
};

#endif // PROCESSCONTROLLER_H
