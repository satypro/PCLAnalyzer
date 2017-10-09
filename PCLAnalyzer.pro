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
    Neighbours/searchneighbour.cpp \
    result.cpp \
    Compute/pclcompute.cpp \
    descriptorprocessor.cpp

HEADERS  += pclanalyzerwindow.h \
    Neighbours/types.h \
    Neighbours/searchneighbour.h \
    result.h \
    datatypes.h \
    Compute/compute.h \
    Compute/pclcompute.h \
    descriptorprocessor.h \
    descriptorprocessorbase.h \
    Neighbours/searchneighbourbase.h

FORMS    += pclanalyzerwindow.ui

DISTFILES +=

INCLUDEPATH += /usr/local/include /usr/local/include/pcl-1.7 /usr/include/eigen3 /usr/include/teem /usr/include/vtk-5.10

QMAKE_LIBDIR += /usr/local/lib /usr/lib /usr/lib/libteem /usr/local/include
LIBS += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_filters -lpcl_visualization
LIBS += -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_search -lGL -lGLU
LIBS += -lteem
LIBS += -L"/usr/include" -lCGAL -lgmp
