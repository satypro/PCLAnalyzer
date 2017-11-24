#-------------------------------------------------
#
# Project created by QtCreator 2017-10-04T10:52:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCLAnalyzer
TEMPLATE = app


SOURCES += main.cpp\
        pclanalyzerwindow.cpp \
    result.cpp \
    Neighbours/SearchNeighbourOctTree.cpp \
    Neighbours/SearchNeighbourFactory.cpp \
    Descriptors/DescriptorFactory.cpp \
    Descriptors/Descriptor.cpp \
    Utilities/CommonUtility.cpp \
    Classifiers/Classifier.cpp \
    Classifiers/ClassifiersBase.cpp \
    Classifiers/ClassifiersFactory.cpp \
    Config/Configuration.cpp \
    mainwindow.cpp \
    IO/FileRead.cpp \
    Controllers/MainController.cpp \
    Views/ViewFactory.cpp \
    Views/PCLView.cpp \
    Controllers/ProcessController.cpp \
    Controllers/ControllerFactory.cpp

HEADERS  += pclanalyzerwindow.h \
    result.h \
    Neighbours/SearchNeighbourOctTree.h \
    Neighbours/SearchNeighbourFactory.h \
    Descriptors/DescriptorFactory.h \
    Descriptors/DescriptorBase.h \
    Descriptors/Descriptor.h \
    Types/datatypes.h \
    Neighbours/SearchOptions.h \
    Neighbours/SearchNeighbourBase.h \
    Utilities/CommonUtility.h \
    Classifiers/Classifier.h \
    Classifiers/ClassifiersBase.h \
    Classifiers/ClassifiersFactory.h \
    Classifiers/ClassifierLabels.h \
    Classifiers/ClassifierType.h \
    Config/Configuration.h \
    mainwindow.h \
    IO/FileRead.h \
    Utilities/eig3.h \
    Controllers/MainController.h \
    Views/ViewFactory.h \
    Views/IViews.h \
    Models/ViewModel.h \
    Views/PCLView.h \
    Controllers/ProcessController.h \
    Controllers/IControllerBase.h \
    Controllers/ControllerFactory.h

FORMS    += pclanalyzerwindow.ui \
    mainwindow.ui

DISTFILES += \
    config.json

INCLUDEPATH += /usr/local/include /usr/local/include/pcl-1.8 /usr/include/eigen3 /usr/include/teem /usr/include/vtk-5.10 /usr/include/jsoncpp

QMAKE_LIBDIR += /usr/local/lib /usr/lib /usr/lib/libteem /usr/local/include
LIBS += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_filters -lpcl_visualization
LIBS += -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_search -lGL -lGLU
LIBS += -lteem
LIBS += -L"/usr/include" -lCGAL -lgmp
