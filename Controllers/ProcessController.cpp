#include "ProcessController.h"
#include <pcl/visualization/cloud_viewer.h>
#include "Neighbours/SearchOptions.h"
#include "Neighbours/SearchNeighbourOctTree.h"
#include "Neighbours/SearchNeighbourFactory.h"
#include "Classifiers/ClassifiersFactory.h"
#include "Classifiers/ClassifierLabels.h"
#include "Classifiers/ClassifierType.h"
#include "Utilities/CommonUtility.h"
#include "IO/FileRead.h"
#include "Models/ViewModel.h"
#include <math.h>
#include "boost/lexical_cast.hpp"

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

    // Get the Strategy to select the neighbours of a point
    /*Setting the Search Scale*/
    SearchParameter parameter;
    parameter.radius = 0.01f;
    parameter.kNearest = 20;
    parameter.resolution = 128.0f; // Get this value from UI setting

    /*Search Neighbour Options*/
    SearchOption option;
    option.neighbourSearchDataStructure = OctTree; // Get this Vlaue from the DropDown
    option.neighbourSearchTypes = Radius;
    option.searchParameter = parameter;

    SearchNeighbourBase* search = GetNeighbourSearchStrategy(option);
    search->Build(cloud);

    // Then decide how to classify the points.
    ClassifiersBase* classifier = GetClassifier(request["classifierType"], request);
    classifier->SetSearchStrategy(search);
    classifier->setCloud(cloud);

    // 1. For each point in the cloud classify the point.
    size_t size = cloud->points.size ();
    model->cloud = cloud;
    for (size_t i = 0; i < size; ++i)
    {
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }
        
        classifier->setSource(cloud->points[i]);        
        IPointDescriptor* pointdescriptor = classifier->Classify();
        model->descriptor.push_back(pointdescriptor);

        std::cout<<"Progress : "<<ceil(((float)i/(float)size)*100)<<"%"<<std::endl;
    }

    return model;
}

SearchNeighbourBase* ProcessController::GetNeighbourSearchStrategy(SearchOption option)
{
    SearchNeighbourBase* search =
            SearchNeighbourFactory::GetNeighbourSearchDataStructure(option.neighbourSearchDataStructure);
    search->searchOption = option;

    return search;
}

ClassifiersBase* ProcessController::GetClassifier(std::string classifierSelected, std::map<std::string, std::string> request)
{
    ClassifierTypes classifierType = static_cast<ClassifierTypes>
            (boost::lexical_cast<int>(classifierSelected));

    ClassifiersBase* classifier = ClassifiersFactory::GetClassifier(classifierType);

    Configuration* classifierConfig = classifier->GetConfig();

    std::map<std::string, std::string>& classifierParameter = classifierConfig->GetConfig();
    /*MOVE THIS TO THE CODE AS A PRIVATE CONST PROPERTIESE*/
    classifierParameter["sigma"] = "1.0";
    classifierParameter["lambdaN"] = "1.0";
    classifierParameter["delta"] = "0.16";
    classifierParameter["radius"] = "0.01";

    /*THESE COME FROM THE UI */
    classifierParameter["epsi"] = request["epsi"];
    classifierParameter["sigmin"] = request["sigmin"];
    classifierParameter["sigmax"] = request["sigmax"];
    classifierParameter["scalarmin"] = request["scalarmin"];
    classifierParameter["scalarmax"] = request["scalarmax"];

    classifierParameter["rmin"] = request["rmin"];
    classifierParameter["rmax"] = request["rmax"];
    classifierParameter["scale"] = request["scale"];

    classifierParameter["classifierType"] = request["classifierType"];

    return classifier;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProcessController::ReadPointCloud(std::string filePath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tempCloud (new pcl::PointCloud<pcl::PointXYZ>);

    return _tempCloud;
}
