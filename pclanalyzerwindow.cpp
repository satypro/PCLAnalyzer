#include "pclanalyzerwindow.h"
#include "ui_pclanalyzerwindow.h"
#include <iostream>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QTextEdit>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
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
    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select File"), "/home/", tr("Files (*.*)"));
    txt->setText(fileName);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName.toUtf8().constData(), *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file some_pcd.pcd \n");
            return;
    }

    /*Setting the Search Scale*/
    SearchScale scale;
    scale.radius = 0.5f;
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
    ClassifiersBase* classifier = ClassifiersFactory::GetClassifier(BasicClassifier);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbourCloud = search->GetNeighbourCloud(cloud->points[i], option);
        descriptor->setCloud(neighbourCloud);

        std::vector<ClassLabels> labels = classifier->Classify(descriptor);

        std::cout<<"The Point at : "<<i<<" classified as : "<<labels.at(0)<<std::endl;

        /*
        Eigen::Matrix3f covariance_matrix = descriptor->ComputeCovarianceMatrix();
        std::cout<<"The CoVariance Matrix : "<<i<<std::endl;
        std::cout<<covariance_matrix(0,0) << " "<<covariance_matrix(0,1) <<" "<<covariance_matrix(0,2)<<std::endl;
        std::cout<<covariance_matrix(1,0) << " "<<covariance_matrix(1,1) <<" "<<covariance_matrix(1,2)<<std::endl;
        std::cout<<covariance_matrix(2,0) << " "<<covariance_matrix(2,1) <<" "<<covariance_matrix(2,2)<<std::endl;

        EigenResult result = CommonUtility::ComputeEigen(covariance_matrix);
        std::cout<<"The Eigen Values for Matrix : "<<i<<std::endl;
        std::cout<<result.EigenValues(0,0) << " "<<result.EigenValues(1,0) <<" "<<result.EigenValues(2,0)<<std::endl;
        */
    }
    /*One of the way to get the descriptors*/
    /*Using Decriptor for each point infer the  point structure, like if its a part of line, surface etc.*/
}

PCLAnalyzerWindow::~PCLAnalyzerWindow()
{
    delete ui;
}
