#ifndef PARAMETERWIDGET_H
#define PARAMETERWIDGET_H

#include <QWidget>
#include <QMainWindow>
#include <QAction>
#include <QtGui>
#include "pclanalyzerwindow.h"

class QSlider;
class DoubleSlider;

class ParameterWidget : public QWidget
{
    Q_OBJECT

public:
    ParameterWidget(PCLAnalyzerWindow *mainWnd);

public slots:
    void rminChanged(double value);
    void rmaxChanged(double value);
    void scaleChanged(int value);

private:
    PCLAnalyzerWindow *mainWnd;
    DoubleSlider* CreateRadiusSlider(int value);
    QSlider* CreateScaleSlider();

    DoubleSlider* rminSlider;
    DoubleSlider* rmaxSlider;
    QSlider* scaleSlider;

    QLabel* rminLabel;
    QLabel* rmaxLabel;
    QLabel* scaleLabel;
};

#endif // PARAMETERWIDGET_H
