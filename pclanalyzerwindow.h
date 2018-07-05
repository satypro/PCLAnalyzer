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
#include "Classifiers/ClassifierType.h"

namespace Ui {
class PCLAnalyzerWindow;
}

class GLWidget;
class QComboBox;
class StructureParameterWidget;
class ParameterWidget;

class PCLAnalyzerWindow : public QMainWindow
{
    Q_OBJECT

public:
    //PCLAnalyzerWindow();
    explicit PCLAnalyzerWindow(QWidget *parent = 0);
    void display();
    ~PCLAnalyzerWindow();
public slots:
    void SetFilePath();
    void ProcessCloud();
    void SetRmin(double rmin);
    void SetRmax(double rmax);
    void SetScale(int scale);
    void SetEps(double eps);
    void SetSigmaMin(double sigmin);
    void SetSigmaMax(double sigmax);
    void SetScalarMin(double scalarMin);
    void SetScalarMax(double scalarMax);
    void SetTensorType(int index);
    void SetDisplayMode(int displaymode, int pointmode);
    void SetMeatFeatDispMode(int pointMode);

private:
    Ui::PCLAnalyzerWindow *ui;
    QTextEdit *txt;
    GLWidget *glWidget;
    ParameterWidget* parameterWidget;
    StructureParameterWidget* structreParameterWidget;
    std::string _filePath;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<ClassLabels> classLabels;
    bool displayCloud = false;
    IViews* _view;
    QComboBox *ftdisp2;
    QComboBox* tensorType;
    float eps;
    float sigmin;
    float sigmax;
    float scalarMin;
    float scalarMax;
    float rmin;
    float rmax;
    int scale;
    int _displaymode, _pointmode;
    ClassifierTypes classifierType;

    std::map<std::string, std::string>& PrepareRequest();

};

#endif // PCLANALYZERWINDOW_H
