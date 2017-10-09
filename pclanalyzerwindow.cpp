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
#include "Neighbours/types.h"
#include "Neighbours/searchneighbourbase.h"
#include "Neighbours/searchneighbour.h"
#include "descriptorprocessorbase.h"
#include "descriptorprocessor.h"

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
    scale.resolution = 128.0f;
    /*Search Neighbour Routine*/
    SearchNeighbourBase* search =  new SearchNeighbour();

    /*One of the way to get the descriptors*/
    DescriptorProcessorBase* obj = new DescriptorProcessor(search);
    std::vector<Result*> result = obj->getDescriptors(cloud, scale, Radius);

    /*Using Decriptor for each point infer the  point structure, like if its a part of line, surface etc.*/
}

PCLAnalyzerWindow::~PCLAnalyzerWindow()
{
    delete ui;
}
