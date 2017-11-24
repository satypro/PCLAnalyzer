#include "pclanalyzerwindow.h"
#include "ui_pclanalyzerwindow.h"
#include <iostream>
#include <vector>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QTextEdit>
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

PCLAnalyzerWindow::PCLAnalyzerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLAnalyzerWindow)
{
    ui->setupUi(this);
    
    QPushButton *btnGetFile = new QPushButton(this);
    txt = new QTextEdit();
    
    btnGetFile->setText("Select File");
    btnGetFile->setFixedWidth(100);
    btnGetFile->setFixedHeight(30);
    txt->setFixedHeight(30);
    
    QObject::connect(btnGetFile, SIGNAL(clicked()),this, SLOT(clickedSlot()));
    btnGetFile->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    
    QWidget* centralWidget = new QWidget(this);
    centralWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    
    QVBoxLayout* layout = new QVBoxLayout(centralWidget);
    QHBoxLayout* hLayout = new QHBoxLayout(centralWidget);
    hLayout->addWidget(txt);
    hLayout->addWidget(btnGetFile);
    layout->addLayout(hLayout);
    
    setCentralWidget(centralWidget);
    setWindowTitle("Spatial Partitioning");
}

void PCLAnalyzerWindow::clickedSlot()
{
    std::vector <float> _intensity;

    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select File"), "/home/", tr("Files (*.*)"));
    txt->setText(fileName);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    /*
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toUtf8().constData(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
            return;
    }
    */

    std::string _filePath = fileName.toUtf8().constData();

    if(_filePath == "NULL")
    {
        std::cout<<"Invalid File Path"<<std::endl;
        return;
    }
    else
    {
        FileRead *readObject = new FileRead;
        bool nErrorCode = readObject->read(_filePath, cloud, _intensity);
        delete readObject;

        if (nErrorCode == false)
            return;
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
    /*Get the config for the Types and show this as the Property in the windows*/
    Configuration* config = search->GetConfig();
    std::map<std::string, std::string>& mapp = config->GetConfig();

    /*for(std::map<std::string, std::string>::iterator it = mapp.begin(); it!=mapp.end(); ++it )
    {
        std::cout<<it->first<<" "<<it->second<<std::endl;
    }
    */
    mapp["Radius"] = "1";
    //config->SetValue("Radius","12");

    DescriptorBase* descriptor =  DescriptorFactory::GetDescriptor();
    Configuration* descriptorConfig = descriptor->GetConfig();
    std::map<std::string, std::string>& descriptorProperty = descriptorConfig->GetConfig();
    descriptorProperty["sigma"] = "1.0";
    descriptorProperty["lambdaN"] = "1.0";

    ClassifiersBase* classifier = ClassifiersFactory::GetClassifier(BasicClassifier);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colordCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
        {
            std::cout<<"The Point at : "<<i<<" NAN : "<<std::endl;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = search->GetNeighbourCloud(cloud->points[i], option);
        descriptor->setSource(cloud->points[i]);
        descriptor->setCloud(neighbourCloud);

        std::vector<ClassLabels> labels = classifier->Classify(descriptor);

        std::cout<<"The Point at : "<<i<<" classified as : "<<labels.at(0)<<std::endl;

        if (labels.at(0) == Point)
        {
            pcl::PointXYZRGB point = pcl::PointXYZRGB(255,0, 0);
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            colordCloud->points.push_back(point);
        }

        if (labels.at(0) == Curve)
        {
            pcl::PointXYZRGB point = pcl::PointXYZRGB(0,255, 0);
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            colordCloud->points.push_back(point);
        }

        if (labels.at(0) == Disc)
        {
            pcl::PointXYZRGB point = pcl::PointXYZRGB(0,0, 255);
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            colordCloud->points.push_back(point);
        }
    }

    pcl::visualization::CloudViewer viewer ("Colored Viewver");
    viewer.showCloud(colordCloud);

    while(!viewer.wasStopped())
    {}
}

PCLAnalyzerWindow::~PCLAnalyzerWindow()
{
    delete ui;
}
