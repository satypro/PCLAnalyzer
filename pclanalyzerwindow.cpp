#include "pclanalyzerwindow.h"
#include "ui_pclanalyzerwindow.h"
#include <iostream>
#include <vector>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QProgressDialog>
#include <QTextEdit>
#include <QtOpenGL>
#include "Display/Glwidget.h"
#include "Controllers/MainController.h"
#include "Config/Request.h"
#include "UI/Component/StructureParameterWidget.h"
#include "UI/Widgets/ParameterWidget.h"
#include "boost/lexical_cast.hpp"
#include "Views/PCLView.h"

PCLAnalyzerWindow::PCLAnalyzerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLAnalyzerWindow)
{
    glWidget = new GLWidget(0, this);
    this->resize(800, 800);
    setCentralWidget(glWidget);
    setWindowTitle(tr("Point Cloud Visualization Board"));

    QPushButton *btnGetFile = new QPushButton(this);
    btnGetFile->setText("Select File");
    btnGetFile->setFixedWidth(100);
    btnGetFile->setFixedHeight(30);

    txt = new QTextEdit();
    txt->setFixedHeight(30);
    
    QObject::connect(btnGetFile, SIGNAL(clicked()),this, SLOT(SetFilePath()));
    btnGetFile->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

    QAction *action_Open = new QAction(this);
    action_Open->setText(tr("Open"));
    action_Open->setShortcut(tr("Ctrl+F"));
    action_Open->setStatusTip(tr("Open file"));
    connect(action_Open, SIGNAL(triggered()), this, SLOT(SetFilePath()));

    QAction *exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    QMenu *fileMenu = fileMenu = menuBar()->addMenu(tr("&File"));

    fileMenu->addAction(action_Open);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    tensorType = new QComboBox();
    // Fill the items of the ComboBox
    tensorType->addItem("3DVT-GET");
    tensorType->addItem("3DVT");
    tensorType->addItem("3DCM");
    tensorType->addItem("3DMCM");
    tensorType->addItem("2DGET");
    tensorType->addItem("Hessian");
    tensorType->addItem("2DCM");
    connect(tensorType, SIGNAL (currentIndexChanged(int)), this, SLOT (SetTensorType(int)));

    setStyleSheet(QString::fromUtf8("QGroupBox { border: 2px solid red; margin-bottom: 7px;margin-right: 7px; padding: 5px} QGroupBox::title {top:7 ex;left: 10px; subcontrol-origin: border}"));
    QGroupBox *previewGroupBox;
    previewGroupBox = new QGroupBox(tr("Parameters"));

    QGroupBox *structureParameterGroupBox;
    structureParameterGroupBox = new QGroupBox(tr("Structure Classificaton Hyper Parameters"));

    parameterWidget = new ParameterWidget(this);
    QGridLayout *previewLayout = new QGridLayout;
    previewLayout->addWidget(parameterWidget);
    previewLayout->addWidget(tensorType);
    previewGroupBox->setLayout(previewLayout);

    QPushButton *btnReadCloud = new QPushButton(this);
    btnReadCloud->setText("Process Cloud");
    btnReadCloud->setFixedWidth(100);
    btnReadCloud->setFixedHeight(30);
    QObject::connect(btnReadCloud, SIGNAL(clicked()),this, SLOT(ProcessCloud()));
    btnReadCloud->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

    //QWidget* centralWidget = new QWidget(this);
    //centralWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    
    QVBoxLayout* structureClassifierlayout = new QVBoxLayout;

    structreParameterWidget = new StructureParameterWidget(this);
    QHBoxLayout* structureParameterLayout = new QHBoxLayout;
    structureParameterLayout->addWidget(structreParameterWidget);

    QHBoxLayout* hLayout = new QHBoxLayout;
    hLayout->addWidget(txt);
    hLayout->addWidget(btnGetFile);

    ftdisp2 = new QComboBox();
    // Fill the items of the ComboBox
    ftdisp2->addItem("Intensity");
    ftdisp2->addItem("Saliency");
    ftdisp2->addItem("Surface Variation");
    ftdisp2->addItem("ClCsCp");
    ftdisp2->addItem("Curve Graph");
    ftdisp2->addItem("Sum of Eigenvalues");
    ftdisp2->addItem("Planarity");
    ftdisp2->addItem("Anisotropy");
    ftdisp2->addItem("Sphericity");
    ftdisp2->addItem("Triangulation");
    ftdisp2->addItem("Triangulation Points");
    ftdisp2->addItem("DoN");
    ftdisp2->addItem("Contour");
    ftdisp2->addItem("Tensor Lines");
    ftdisp2->addItem("Height");
    ftdisp2->addItem("Linearity");
    ftdisp2->addItem("Omnivariance");
    ftdisp2->addItem("Eigenentropy");
    ftdisp2->addItem("Labels");

    connect(ftdisp2, SIGNAL (currentIndexChanged(int)), this, SLOT (SetMeatFeatDispMode(int)));

    QHBoxLayout* processCloudLayout = new QHBoxLayout;
    processCloudLayout->addWidget(ftdisp2);
    processCloudLayout->addWidget(btnReadCloud);

    structureClassifierlayout->addLayout(structureParameterLayout);
    structureClassifierlayout->addLayout(hLayout);
    structureClassifierlayout->addLayout(processCloudLayout);

    structureParameterGroupBox->setLayout(structureClassifierlayout);


    QDockWidget *dock;
    dock = new QDockWidget(tr(""), this);
    dock->setAllowedAreas(Qt::RightDockWidgetArea);
    dock->setFloating(false);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    dock->setWidget(structureParameterGroupBox);

    QDockWidget *dock1;
    dock1 = new QDockWidget(tr(""), this);
    dock1->setAllowedAreas(Qt::RightDockWidgetArea);
    dock1->setFloating(false);
    addDockWidget(Qt::RightDockWidgetArea, dock1);
    dock1->setWidget(previewGroupBox);
}

// Method is invoked by OpenGL Widget to display in ViewPort
void PCLAnalyzerWindow::display()
{
    if (displayCloud)
    {
       // _view->Show();
        PCLView * pclView = static_cast<PCLView*>(_view);
        if(_displaymode == 0 || (_displaymode == 1 && _pointmode == 0))
        {
               pclView->lasDisplay();
        }
        else if(_displaymode == 1 && _pointmode == 4)
        {
            pclView->displayGraph();
            pclView->displayBoundary();
        }
        else if(_displaymode == 1)
            pclView->renderStructFDs(_pointmode);
    }
}

PCLAnalyzerWindow::~PCLAnalyzerWindow()
{
    delete ui;
}

void PCLAnalyzerWindow::SetRmin(double rmin)
{
    if(rmin != this->rmin)
    {
        this->rmin = rmin;
    }
}

void PCLAnalyzerWindow::SetRmax(double rmax)
{
    if(rmax != this->rmax)
    {
        this->rmax = rmax;
    }
}

void PCLAnalyzerWindow::SetScale(int scale)
{
    if(scale != this->scale)
    {
        this->scale = scale;
    }
}

void PCLAnalyzerWindow::SetEps(double eps)
{
    if(eps != this->eps)
    {
        this->eps = eps;
    }
}

void PCLAnalyzerWindow::SetSigmaMin(double sigmin)
{
    {
        if(sigmin != this->sigmin)
        this->sigmin = sigmin;
    }
}

void PCLAnalyzerWindow::SetSigmaMax(double sigmax)
{
    if(sigmax != this->sigmax)
    {
        this->sigmax = sigmax;
    }
}

void PCLAnalyzerWindow::SetScalarMin(double scalarMin)
{
    if(scalarMin != this->scalarMin)
    {
        this->scalarMin = scalarMin;
    }
}

void PCLAnalyzerWindow::SetScalarMax(double scalarMax)
{
    if(scalarMax != this->scalarMax)
    {
        this->scalarMax = scalarMax;
    }
}

void PCLAnalyzerWindow::SetTensorType(int index)
{
    this->classifierType = static_cast<ClassifierTypes>(index);
}

void PCLAnalyzerWindow::ProcessCloud()
{
    QProgressDialog* progress = new QProgressDialog("Task in Progreess","Cancel", 0, 100);
    progress->setWindowModality(Qt::WindowModal);

    displayCloud = false;

    std::map<std::string, std::string>& request = PrepareRequest();
    progress->setValue(10);
    MainController* controller = new MainController();
    _view = controller->Process("PROCESS", "PCLVIEWER", request);
    progress->setValue(80);
    // View is ready with the model
    // now it can be vizualized
    displayCloud = true;
    progress->setValue(100);
}

void PCLAnalyzerWindow::SetFilePath()
{
    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select File"), "/home/", tr("Files (*.*)"));
    txt->setText(fileName);
    _filePath = fileName.toUtf8().constData();
}

std::map<std::string, std::string>& PCLAnalyzerWindow::PrepareRequest()
{
    Request* req = new Request();
    std::map<std::string, std::string>& request = req->GetRequest();
    request["filePath"] = _filePath;

    // Setting the Structural Classifier Parameters
    request["epsi"] = boost::lexical_cast<std::string>(this->eps);
    request["sigmin"] = boost::lexical_cast<std::string>(this->sigmin);
    request["sigmax"] = boost::lexical_cast<std::string>(this->sigmax);
    request["scalarmin"] = boost::lexical_cast<std::string>(this->scalarMin);
    request["scalarmax"] = boost::lexical_cast<std::string>(this->scalarMax);

    // Setting the General Parameter
    request["rmin"] = boost::lexical_cast<std::string>(this->rmin); // available since c++11
    request["rmax"] = boost::lexical_cast<std::string>(this->rmax);
    request["scale"] = boost::lexical_cast<std::string>(this->scale);

    // Selected Classifier
    request["classifierType"] = boost::lexical_cast<std::string>(this->classifierType);

    return request;
}

void PCLAnalyzerWindow::SetDisplayMode(int displaymode, int pointmode)
{
    this->_displaymode = displaymode;
    this->_pointmode = pointmode;
}

void PCLAnalyzerWindow :: SetMeatFeatDispMode(int pointMode)
{
    this->_displaymode = 1;
    this->_pointmode = pointMode;

    glWidget->setDisplayMode(1);
    glWidget->setPointMode(pointMode);
    glWidget->updateGL();
}
