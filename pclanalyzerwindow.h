#ifndef PCLANALYZERWINDOW_H
#define PCLANALYZERWINDOW_H

#include <QMainWindow>
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
#include "Classifiers/ClassifiersBase.h"
#include "Classifiers/ClassifiersFactory.h"
#include "Classifiers/ClassifierLabels.h"
#include "Classifiers/ClassifierType.h"
#include "Utilities/CommonUtility.h"
#include "Views/IViews.h"

namespace Ui {
class PCLAnalyzerWindow;
}

class GLWidget;

class PCLAnalyzerWindow : public QMainWindow
{
    Q_OBJECT

public:
    //PCLAnalyzerWindow();
    explicit PCLAnalyzerWindow(QWidget *parent = 0);
    void display();
    ~PCLAnalyzerWindow();
public slots:
    void setFilePath();
    void processCloud();

private:
    Ui::PCLAnalyzerWindow *ui;
    QTextEdit *txt;
    GLWidget *glWidget;
    std::string _filePath;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<ClassLabels> classLabels;
    bool displayCloud = false;
    IViews* _view;
};

#endif // PCLANALYZERWINDOW_H
