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
#include "Descriptors/Descriptor.h"
#include "Descriptors/DescriptorBase.h"
#include "Descriptors/DescriptorFactory.h"
#include "Classifiers/ClassifiersBase.h"
#include "Classifiers/ClassifiersFactory.h"
#include "Classifiers/ClassifierLabels.h"
#include "Classifiers/ClassifierType.h"
#include "Utilities/CommonUtility.h"
#include "IO/FileRead.h"
#include <math.h>

ProcessController::ProcessController()
{
}

ViewModel ProcessController::Process(std::map<std::string, std::string> request)
{
    std::vector <float> _intensity;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::string _filePath =  request["filePath"];
    ViewModel model;

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

    DescriptorBase* descriptor =  DescriptorFactory::GetDescriptor();
    Configuration* descriptorConfig = descriptor->GetConfig();
    std::map<std::string, std::string>& descriptorProperty = descriptorConfig->GetConfig();
    descriptorProperty["sigma"] = "1.0";
    descriptorProperty["lambdaN"] = "1.0";
    descriptorProperty["delta"] = "0.16";
    descriptorProperty["epsi"] = "0.5";
    descriptorProperty["rmin"] = "0.1";
    descriptorProperty["rmax"] = "0.2";
    descriptorProperty["radius"] = "0.01";

    ClassifiersBase* classifier = ClassifiersFactory::GetClassifier(BasicClassifier);


    model.cloud = cloud;

    size_t size = cloud->points.size ();

    for (size_t i = 0; i < size; ++i)
    {
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = search->GetNeighbourCloud(cloud->points[i], option);
        descriptor->setSource(cloud->points[i]);
        descriptor->setCloud(neighbourCloud);

        PointDescriptor pointdescriptor = classifier->Classify(descriptor);
        std::cout<<"Progress : "<<ceil(((float)i/(float)size)*100)<<"%"<<std::endl;

        pointdescriptor.label = pointdescriptor.label;
        model.descriptor.push_back(pointdescriptor);
    }

    return model;
}
