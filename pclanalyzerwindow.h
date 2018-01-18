#ifndef PCLANALYZERWINDOW_H
#define PCLANALYZERWINDOW_H

#include <QMainWindow>
#include <QTextEdit>

namespace Ui {
class PCLAnalyzerWindow;
}

class GLWidget;

class PCLAnalyzerWindow : public QMainWindow
{
    Q_OBJECT

public:
    //PCLAnalyzerWindow();
    explicit PCLAnalyzerWindow(QWidget *parent = 0);
    void display();
    ~PCLAnalyzerWindow();
public slots:
    void clickedSlot();

private:
    Ui::PCLAnalyzerWindow *ui;
    QTextEdit *txt;
    GLWidget *glWidget;
};

#endif // PCLANALYZERWINDOW_H
