#ifndef PCLANALYZERWINDOW_H
#define PCLANALYZERWINDOW_H

#include <QMainWindow>
#include <QTextEdit>

namespace Ui {
class PCLAnalyzerWindow;
}

class PCLAnalyzerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLAnalyzerWindow(QWidget *parent = 0);
    ~PCLAnalyzerWindow();
public slots:
    void clickedSlot();

private:
    Ui::PCLAnalyzerWindow *ui;
    QTextEdit *txt;
};

#endif // PCLANALYZERWINDOW_H
