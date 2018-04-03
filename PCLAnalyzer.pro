#-------------------------------------------------
#
# Project created by QtCreator 2017-10-04T10:52:41
#
#-------------------------------------------------

QT       += core \
        gui \
        opengl \
        network \
        xml \
        widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCLAnalyzer
TEMPLATE = app


SOURCES += main.cpp\
        pclanalyzerwindow.cpp \
    Neighbours/SearchNeighbourOctTree.cpp \
    Neighbours/SearchNeighbourFactory.cpp \
    Utilities/CommonUtility.cpp \
    Classifiers/Classifier.cpp \
    Classifiers/ClassifiersBase.cpp \
    Classifiers/ClassifiersFactory.cpp \
    Config/Configuration.cpp \
    IO/FileRead.cpp \
    Controllers/MainController.cpp \
    Views/ViewFactory.cpp \
    Views/PCLView.cpp \
    Controllers/ProcessController.cpp \
    Controllers/ControllerFactory.cpp \
    Display/FastTrackball.cpp \
    Display/Glwidget.cpp \
    Config/Request.cpp \
    Classifiers/Tensor2DClassifier.cpp \
    Classifiers/Tensor3DVotingClassifier.cpp \
    Classifiers/ModCovarianceMatrixClassifier.cpp \
    Classifiers/DiffusedNormalVotingClassifier.cpp \
    Classifiers/CovarianceMatrix2DClassifier.cpp \
    Classifiers/CovarianceMatrixClassifier.cpp \
    Classifiers/BoundaryTensorClassifier.cpp \
    UI/Component/DoubleSlider.cpp \
    UI/Component/StructureParameterWidget.cpp \
    UI/Widgets/ParameterWidget.cpp

HEADERS  += pclanalyzerwindow.h \
    Neighbours/SearchNeighbourOctTree.h \
    Neighbours/SearchNeighbourFactory.h \
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
    IO/FileRead.h \
    Utilities/eig3.h \
    Controllers/MainController.h \
    Views/ViewFactory.h \
    Views/IViews.h \
    Models/ViewModel.h \
    Views/PCLView.h \
    Controllers/ProcessController.h \
    Controllers/IControllerBase.h \
    Controllers/ControllerFactory.h \
    Display/FastTrackball.h \
    Display/Glwidget.h \
    Config/Request.h \
    Models/IViewModel.h \
    Descriptors/IPointDescriptor.h \
    Descriptors/PointDescriptor.h \
    Classifiers/Tensor2DClassifier.h \
    Classifiers/Tensor3DVotingClassifier.h \
    Classifiers/ModCovarianceMatrixClassifier.h \
    Classifiers/DiffusedNormalVotingClassifier.h \
    Classifiers/CovarianceMatrix2DClassifier.h \
    Classifiers/CovarianceMatrixClassifier.h \
    Classifiers/BoundaryTensorClassifier.h \
    UI/Component/DoubleSlider.h \
    UI/Component/StructureParameterWidget.h \
    UI/Widgets/ParameterWidget.h

FORMS    += pclanalyzerwindow.ui \
    mainwindow.ui

DISTFILES += \
    config.json

INCLUDEPATH += /usr/local/include /usr/local/include/pcl-1.8 /usr/include/eigen3 /usr/local/include/teem /usr/include/vtk-5.10 /usr/include/jsoncpp

QMAKE_LIBDIR += /usr/local/lib /usr/lib /usr/local/lib/libteem /usr/local/include
LIBS += -lteem
LIBS += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_filters -lpcl_visualization
LIBS += -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_search -lGL -lGLU
LIBS += -L"/usr/include" -lCGAL -lgmp
