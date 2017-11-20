#include "pclanalyzerwindow.h"
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PCLAnalyzerWindow w;
    w.show();

    //MainWindow ww;
    //ww.show();

    return a.exec();
}
