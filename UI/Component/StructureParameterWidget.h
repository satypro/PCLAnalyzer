#ifndef STRUCTUREPARAMETERWIDGET_H
#define STRUCTUREPARAMETERWIDGET_H
#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QAction>
#include <QtGui>
#include "pclanalyzerwindow.h"

class QSlider;
class DoubleSlider;

class StructureParameterWidget : public QWidget
{
    Q_OBJECT

public:
    StructureParameterWidget(PCLAnalyzerWindow *mainWnd);
protected:

public slots:
    void EpsChanged(double value);
    void SigminChanged(double value);
    void SigmaxChanged(double value);
    void ScalarMinChanged(double value);
    void ScalarMaxChanged(double value);

private:
    PCLAnalyzerWindow* mainWnd;
    DoubleSlider* CreateEpsillonSlider();
    DoubleSlider* CreateSigmaSlider();

    DoubleSlider* epsSlider;
    DoubleSlider* sigminSlider;
    DoubleSlider* sigmaxSlider;
    DoubleSlider* scalarMinSlider;
    DoubleSlider* scalarMaxSlider;

    QLabel* epsLabel;
    QLabel* sigminLabel;
    QLabel* sigmaxLabel;
    QLabel* scalarMinLabel;
    QLabel* scalarMaxLabel;
};

#endif // STRUCTUREPARAMETERWIDGET_H
