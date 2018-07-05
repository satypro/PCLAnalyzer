#include "ProcessController.h"
#include <pcl/visualization/cloud_viewer.h>
#include "Neighbours/SearchOptions.h"
#include "Neighbours/SearchNeighbourOctTree.h"
#include "Neighbours/SearchNeighbourFactory.h"
#include "Classifiers/ClassifiersFactory.h"
#include "Classifiers/ClassifierLabels.h"
#include "Classifiers/ClassifierType.h"
#include "Utilities/CommonUtility.h"
#include "Utilities/Graph.h"
#include "Utilities/Util.h"
#include "IO/FileRead.h"
#include "Types/datatypes.h"
#include <math.h>
#include "boost/lexical_cast.hpp"
#include "Descriptors/PointDescriptor.h"
#include <vector>
#define LABEL_FILE "/home/satendra/data/label/d4-alg2l.txt"

bool comparede(double i, double j)
{
    return i<j;
}

bool myfunction (std::pair<int, double>i, std::pair<int, double> j)
{
    return ( (i.second < j.second));
}

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

        std::string labelFile = LABEL_FILE;
        readObject->readLabels(labelFile, labels);
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
    model->intensity = _intensity;
    model->labels = labels;
    model->descriptor = classifier->Classify();

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        model->descriptor[i]->labels = labels[i];
    }

    char* pEnd;
    float _rmin = ::strtof(request["rmin"].c_str(), &pEnd);
    float _rmax = ::strtof(request["rmax"].c_str(), &pEnd);

    Util * util = new Util();
    util->_inCloud = cloud;
    util->SetSearchStrategy(search);
    util->ComputeDoNs_MSO(model->descriptor,_rmin, _rmax);
    util->Triangulate(_rmin, model->descriptor);
    util->GenerateTensorLines3(model->descriptor);
    util->PruneTriangles(_rmin, model->descriptor);

    StructFeatClassification(model);
    CurveGraphExtraction(model, request);

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

             //pointDescriptor->PtsProp = _ftPtsProp;
         }

         model->PtsProp = _ftPtsProp;

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

void ProcessController::CurveGraphExtraction(ViewModel* model, std::map<std::string, std::string> request)
{
    std::vector<std::pair<int, double> > featureNode;
    std::vector<std::pair<int, double> > critFeatureNode;
    std::vector<gnode> _graph;
    std::vector<int> _graphIdx;

    float _rmaxpt = 500;

    char* pEnd;
    float _rmax = ::strtof(request["rmax"].c_str(), &pEnd);

    for(size_t i =0; i < model->cloud->points.size(); i++)
    {
        if(model->descriptor[i]->featNode.csclcp[0] > model->descriptor[i]->featNode.csclcp[1] && model->descriptor[i]->featNode.csclcp[0] > model->descriptor[i]->featNode.csclcp[2])
        {
            critFeatureNode.push_back(std::make_pair(i, model->descriptor[i]->featNode.csclcp[0]));
        }

        if(model->descriptor[i]->featNode.csclcp[1] >= model->descriptor[i]->featNode.csclcp[0] && model->descriptor[i]->featNode.csclcp[1] >= model->descriptor[i]->featNode.csclcp[2])
        {
            int idx = i;
            double val =  model->descriptor[i]->featNode.csclcp[1];
            featureNode.push_back(std::make_pair(idx, val));
        }
    }

    std::sort (featureNode.begin(), featureNode.end(), myfunction);

    std::sort (critFeatureNode.begin(), critFeatureNode.end(), myfunction);

    //cout<<"sorting done "<<endl;

    std::vector<node> curveSeeds;

    std::vector<node> critCurveSeeds;

    for(size_t i =0; i < featureNode.size(); i++ )
    {
        node temp;
        temp.idx = featureNode[i].first;
        temp.status = false;
        curveSeeds.push_back(temp);
    }

    for(size_t i =0; i < critFeatureNode.size(); i++ )
    {
        node temp;
        temp.idx = critFeatureNode[i].first;
        temp.status = false;
        critCurveSeeds.push_back(temp);
    }

    critFeatureNode.clear();

    featureNode.clear();

    Graph curveGraphHandle;

    curveGraphHandle.setParams(_rmax, _rmaxpt);
    curveGraphHandle.setNode(curveSeeds);
    curveGraphHandle.setCriticalNode(critCurveSeeds);
    curveGraphHandle.setInputcloud(model->cloud);
    curveGraphHandle.setSaliencyMaps(model->descriptor);
    curveGraphHandle.constructGraph(_graph, _graphIdx);

    cout<<"graph extraction done "<<endl;
    model->graph = _graph;

    ofstream fout("Area4 edgeInfo.txt");
    fout<< _graph.size()<<endl;

    for (size_t i = 0 ; i < _graph.size() ; i++)
    {
        int m = _graph[i].nd.idx;
        fout<<m<<" "<<_graph[i].edge.size();

        for(size_t j = 0; j < _graph[i].edge.size(); j++)
        {
            int n = _graph[i].edge[j].idx;

            fout<<" "<<n;
        }

        fout<<"\n";
    }
    fout.close();
}
