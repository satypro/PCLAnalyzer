#include "pclanalyzerwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PCLAnalyzerWindow w;
    w.show();
    return a.exec();
}
