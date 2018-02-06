#include "pclanalyzerwindow.h"
#include "ui_pclanalyzerwindow.h"
#include <iostream>
#include <vector>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QTextEdit>
#include <QtOpenGL>
#include "Display/Glwidget.h"
#include "Controllers/MainController.h"
#include "Config/Request.h"

PCLAnalyzerWindow::PCLAnalyzerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLAnalyzerWindow)
{
    glWidget = new GLWidget(0, this);
    this->resize(800, 800);
    setCentralWidget(glWidget);
    setWindowTitle(tr("Point Cloud Visualization Board"));

    QPushButton *btnGetFile = new QPushButton(this);
    txt = new QTextEdit();
    
    btnGetFile->setText("Select File");
    btnGetFile->setFixedWidth(100);
    btnGetFile->setFixedHeight(30);
    txt->setFixedHeight(30);
    
    QObject::connect(btnGetFile, SIGNAL(clicked()),this, SLOT(setFilePath()));
    btnGetFile->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    


       QAction *action_Open = new QAction(this);
      // action_Open->setObjectName(QString::fromUtf8("action_Open"));
       action_Open->setText(tr("Open"));
      // action_Open->setIcon(ICON_OPEN);
       action_Open->setShortcut(tr("Ctrl+F"));
       action_Open->setStatusTip(tr("Open file"));
       connect(action_Open, SIGNAL(triggered()), this, SLOT(setFilePath()));

       QAction *exitAct = new QAction(tr("E&xit"), this);
       exitAct->setShortcuts(QKeySequence::Quit);
       connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

       QMenu *fileMenu = fileMenu = menuBar()->addMenu(tr("&File"));

       fileMenu->addAction(action_Open);
       fileMenu->addSeparator();
       fileMenu->addAction(exitAct);


       QComboBox* tensorType = new QComboBox();
           // Fill the items of the ComboBox
           tensorType->addItem("3DVT-GET");
           tensorType->addItem("3DVT");
           tensorType->addItem("3DCM");
           tensorType->addItem("3DMCM");
           tensorType->addItem("2DGET");
           tensorType->addItem("Hessian");
           tensorType->addItem("2DCM");

         QGroupBox *previewGroupBox;
         previewGroupBox = new QGroupBox(tr("Parameters"));
            setStyleSheet(QString::fromUtf8("QGroupBox { border: 2px solid red; margin-bottom: 7px;margin-right: 7px; padding: 5px} QGroupBox::title {top:7 ex;left: 10px; subcontrol-origin: border}"));

        QGridLayout *previewLayout = new QGridLayout;

         previewLayout->addWidget(tensorType);
        previewGroupBox->setLayout(previewLayout);









    QPushButton *btnReadCloud = new QPushButton(this);
    btnReadCloud->setText("Process Cloud");
    btnReadCloud->setFixedWidth(100);
    btnReadCloud->setFixedHeight(30);

    QObject::connect(btnReadCloud, SIGNAL(clicked()),this, SLOT(processCloud()));
    btnReadCloud->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);


    QWidget* centralWidget = new QWidget(this);
    centralWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    
    QVBoxLayout* layout = new QVBoxLayout(centralWidget);
    QHBoxLayout* hLayout = new QHBoxLayout(centralWidget);
    hLayout->addWidget(txt);
    hLayout->addWidget(btnGetFile);

    QHBoxLayout* hLayout2 = new QHBoxLayout(centralWidget);
    hLayout2->addWidget(btnReadCloud);

    layout->addLayout(hLayout);
    layout->addLayout(hLayout2);


    QDockWidget *dock1;
        dock1 = new QDockWidget(tr(""), this);
        dock1->setAllowedAreas(Qt::RightDockWidgetArea);
        dock1->setFloating(false);
        addDockWidget(Qt::RightDockWidgetArea, dock1);
        dock1->setWidget(previewGroupBox);

    QDockWidget *dock;
    dock = new QDockWidget(tr(""), this);
    dock->setAllowedAreas(Qt::RightDockWidgetArea);
    dock->setFloating(false);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    dock->setWidget(centralWidget);



}

void PCLAnalyzerWindow::processCloud()
{
    displayCloud = false;

    Request* req = new Request();
    std::map<std::string, std::string>& request = req->GetRequest();
    request["filePath"] = _filePath;

    MainController* controller = new MainController();
    _view = controller->Process("PROCESS", "PCLVIEWER", request);

    // View is ready with the model
    // now it can be vizualized
    displayCloud = true;
}

void PCLAnalyzerWindow::setFilePath()
{
    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select File"), "/home/", tr("Files (*.*)"));
    txt->setText(fileName);
    _filePath = fileName.toUtf8().constData();
}

// Method is invoked by OpenGL Widget to display in ViewPort
void PCLAnalyzerWindow::display()
{
    if (displayCloud)
    {
        _view->Show();
    }
}

PCLAnalyzerWindow::~PCLAnalyzerWindow()
{
    delete ui;
}
