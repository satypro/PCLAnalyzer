#ifndef DOUBLESLIDER_H
#define DOUBLESLIDER_H
#include <QApplication>
#include <QtGui>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>

class DoubleSlider : public QSlider
{
    Q_OBJECT

public:
    DoubleSlider(QWidget *parent = 0) : QSlider(parent)
    {
        connect(this, SIGNAL(valueChanged(int)),
            this, SLOT(NotifyValueChanged(int)));
    }

signals:
    void DoubleValueChanged(double value);

public slots:
    void NotifyValueChanged(int value) {
        double doubleValue = value / 1000.0;
        emit DoubleValueChanged(doubleValue);
    }
};

#endif // DOUBLESLIDER_H
