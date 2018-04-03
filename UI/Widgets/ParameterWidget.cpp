#include "ParameterWidget.h"
#include "UI/Component/DoubleSlider.h"

ParameterWidget::ParameterWidget(PCLAnalyzerWindow *mainWnd)
{
    this->mainWnd = mainWnd;

    rminLabel = new QLabel("No Data");
    rmaxLabel = new QLabel("No Data");
    scaleLabel = new QLabel("No Data");
    rminSlider = CreateRadiusSlider(100);
    rmaxSlider = CreateRadiusSlider(200);
    scaleSlider = CreateScaleSlider();

    connect(rminSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(rminChanged(double)));
    connect(rmaxSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(rmaxChanged(double)));
    connect(scaleSlider, SIGNAL(valueChanged(int)),this,SLOT(scaleChanged(int)));

    connect(rminSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetRmin(double)));
    connect(rmaxSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetRmax(double)));
    connect(scaleSlider, SIGNAL(valueChanged(int)), mainWnd, SLOT(SetScale(int)));

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *hmainLayout = new QHBoxLayout;
    hmainLayout->addWidget(rminSlider);
    hmainLayout->addWidget(rminLabel);

    QHBoxLayout *hmainLayout1 = new QHBoxLayout;
    hmainLayout1->addWidget(rmaxSlider);
    hmainLayout1->addWidget(rmaxLabel);

    QHBoxLayout *hmainLayout2 = new QHBoxLayout;
    hmainLayout2->addWidget(scaleSlider);
    hmainLayout2->addWidget(scaleLabel);

    mainLayout->addLayout(hmainLayout);
    mainLayout->addLayout(hmainLayout1);
    mainLayout->addLayout(hmainLayout2);
    setLayout(mainLayout);

    rminSlider->setValue(9);
    rmaxSlider->setValue(11);
    scaleSlider->setValue(3);
}

DoubleSlider *ParameterWidget::CreateRadiusSlider(int value)
{
    DoubleSlider *slider = new DoubleSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, value);
    slider->setSingleStep(1000);
   // slider->setPageStep(15 * 16);
   // slider->setTickInterval(10);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

QSlider *ParameterWidget::CreateScaleSlider()
{
    QSlider *slider = new QSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 5);
    slider->setSingleStep(1);
    //slider->setPageStep(15 * 16);
    //slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}


void  ParameterWidget::rminChanged(double value)
{
    rminLabel->setText("rmin: "+ QString::number(value) );
}

void  ParameterWidget::rmaxChanged(double value)
{
    rmaxLabel->setText("rmax: "+ QString::number(value) );
}

void  ParameterWidget::scaleChanged(int value)
{
    scaleLabel->setText("scale: "+ QString::number(value) );
}
