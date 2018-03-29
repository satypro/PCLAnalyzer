#include "ProcessController.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include "Neighbours/SearchOptions.h"
#include "Neighbours/SearchNeighbourBase.h"
#include "Neighbours/SearchNeighbourOctTree.h"
#include "Neighbours/SearchNeighbourFactory.h"
#include "Classifiers/ClassifiersBase.h"
#include "Classifiers/ClassifiersFactory.h"
#include "Classifiers/ClassifierLabels.h"
#include "Classifiers/ClassifierType.h"
#include "Utilities/CommonUtility.h"
#include "IO/FileRead.h"
#include "Models/ViewModel.h"
#include <math.h>

ProcessController::ProcessController()
{
}

IViewModel* ProcessController::Process(std::map<std::string, std::string> request)
{
    std::vector <float> _intensity;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::string _filePath =  request["filePath"];
    ViewModel* model = new ViewModel;

    if(_filePath == "NULL")
    {
        std::cout<<"Invalid File Path"<<std::endl;
        return model;
    }
    else
    {
        FileRead *readObject = new FileRead;
        bool nErrorCode = readObject->read(_filePath, _tempCloud, _intensity);
        delete readObject;

        if (nErrorCode == false)
            return model;

        cloud = _tempCloud;
    }

    /*Setting the Search Scale*/
    SearchScale scale;
    scale.radius = 0.01f;
    scale.kNearest = 20;
    scale.resolution = 128.0f;

    /*Search Neighbour Options*/
    SearchOption option;
    option.neighbourSearchDataStructure = OctTree;
    option.neighbourSearchTypes = Radius;
    option.scale = scale;

    SearchNeighbourBase* search = SearchNeighbourFactory::GetNeighbourSearchDataStructure(OctTree);
    search->Build(cloud, scale.resolution);

    ClassifiersBase* classifier = ClassifiersFactory::GetClassifier(BasicClassifier);
    Configuration* classifierConfig = classifier->GetConfig();
    std::map<std::string, std::string>& classifierParameter = classifierConfig->GetConfig();
    classifierParameter["sigma"] = "1.0";
    classifierParameter["lambdaN"] = "1.0";
    classifierParameter["delta"] = "0.16";
    classifierParameter["epsi"] = "0.5";
    classifierParameter["rmin"] = "0.1";
    classifierParameter["rmax"] = "0.2";
    classifierParameter["radius"] = "0.01";

    model->cloud = cloud;

    size_t size = cloud->points.size ();

    for (size_t i = 0; i < size; ++i)
    {
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = search->GetNeighbourCloud(cloud->points[i], option);

        classifier->setSource(cloud->points[i]);
        classifier->setCloud(neighbourCloud);
        IPointDescriptor* pointdescriptor = classifier->Classify();
        std::cout<<"Progress : "<<ceil(((float)i/(float)size)*100)<<"%"<<std::endl;

        model->descriptor.push_back(pointdescriptor);
    }

    return model;
}
