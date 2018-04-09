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
#include <math.h>
#include "boost/lexical_cast.hpp"
#include "Descriptors/PointDescriptor.h"
#include <vector>

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
    model->cloud = cloud;
    model->descriptor = classifier->Classify();

   /* // 1. For each point in the cloud classify the point.
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
    */

    StructFeatClassification(model);

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

void ProcessController::StructFeatClassification(ViewModel* model)
{
     size_t i, cloudSize = model->cloud->points.size();
     std::vector <unsigned int> _ftPtsProp;

     if(model->descriptor.size() == 0 || cloudSize != model->descriptor.size())
     {
         cout<<"there are no points to classify "<<endl;
                 return;
     }

     _ftPtsProp.resize(cloudSize);

         size_t l_c = 0, l_s =  0, l_p =0;

         float min_cl = 100;
         float min_cp = 100;
         float min_cs = 100;
         float min_anisotropy = 100;
         float min_sphericity = 100;
         float min_don = 100;
         float max_cl = -100;
         float max_cp = -100;
         float max_cs = -100;
         float max_anisotropy = -100;
         float max_sphericity = -100;
         float max_don = -100;
         int numRoofPoints = 0;
         int numRoofTriangulationPoints = 0;

         for(i = 0; i < cloudSize; i++)
         {
             /*if(pointDescriptor->featNode.label==5)
                 numRoofPoints++;
             if(pointDescriptor->featNode.label==5 && pointDescriptor->featNode.triangles.size()>0)
                 numRoofTriangulationPoints++;
             */

             PointDescriptor* pointDescriptor = (PointDescriptor* )model->descriptor[i];

             if(pointDescriptor->featNode.csclcp[0]<min_cs)
                 min_cs = pointDescriptor->featNode.csclcp[0];
             if(pointDescriptor->featNode.csclcp[0]>max_cs)
                 max_cs = pointDescriptor->featNode.csclcp[0];

             if(pointDescriptor->featNode.csclcp[1]<min_cl)
                 min_cl = pointDescriptor->featNode.csclcp[1];
             if(pointDescriptor->featNode.csclcp[1]>max_cl)
                 max_cl = pointDescriptor->featNode.csclcp[1];

             if(pointDescriptor->featNode.csclcp[2]<min_cp)
                 min_cp = pointDescriptor->featNode.csclcp[2];
             if(pointDescriptor->featNode.csclcp[2]>max_cp)
                 max_cp = pointDescriptor->featNode.csclcp[2];

             if(pointDescriptor->featNode.anisotropy<min_anisotropy)
                 min_anisotropy = pointDescriptor->featNode.anisotropy;
             if(pointDescriptor->featNode.anisotropy>max_anisotropy)
                 max_anisotropy = pointDescriptor->featNode.anisotropy;

             if(pointDescriptor->featNode.sphericity<min_sphericity)
                 min_sphericity = pointDescriptor->featNode.sphericity;
             if(pointDescriptor->featNode.sphericity>max_sphericity)
                 max_sphericity = pointDescriptor->featNode.sphericity;

             if(pointDescriptor->featNode.don<min_don)
                 min_don = pointDescriptor->featNode.don;
             if(pointDescriptor->featNode.don>max_don)
                 max_don = pointDescriptor->featNode.don;

             if(pointDescriptor->featNode.csclcp[0] >= pointDescriptor->featNode.csclcp[1] && pointDescriptor->featNode.csclcp[0] >= pointDescriptor->featNode.csclcp[2])
             {
                 _ftPtsProp[i] = 1;
                 l_s++;
             }

             else if(pointDescriptor->featNode.csclcp[1] >= pointDescriptor->featNode.csclcp[0] && pointDescriptor->featNode.csclcp[1] >= pointDescriptor->featNode.csclcp[2])
             {
                 _ftPtsProp[i] = 2;
                 l_c++;
             }

             else if(pointDescriptor->featNode.csclcp[2] >= pointDescriptor->featNode.csclcp[1] && pointDescriptor->featNode.csclcp[2] >= pointDescriptor->featNode.csclcp[0])
             {
                 _ftPtsProp[i] = 3;
                 l_p++;
             }
             else
                  _ftPtsProp[i] = 0;


             pointDescriptor->PtsProp = _ftPtsProp;

         }

         std::cout<<"number of line-type features "<<l_c<<std::endl;
         std::cout<<"number of  critical features "<<l_s<<std::endl;
         std::cout<<"number of surface-type features "<<l_p<<std::endl;
         std::cout <<"min cl = " << min_cl << " max cl = "<< max_cl << std::endl;
         std::cout <<"min cp = " << min_cp << " max cp = "<< max_cp << std::endl;
         std::cout <<"min cs = " << min_cs << " max cs = "<< max_cs << std::endl;
         std::cout <<"min anisotropy = " << min_anisotropy << " max anisotropy = "<< max_anisotropy << std::endl;
         std::cout <<"min sphericity = " << min_sphericity << " max sphericity = "<< max_sphericity << std::endl;
         std::cout <<"min don = " << min_don << " max don = "<< max_don << std::endl;
         std::cout << "# roof points = " << numRoofPoints << " # triangulation roof points = " << numRoofTriangulationPoints << std::endl;
}
