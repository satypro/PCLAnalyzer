#include "SliderWidget.h"
#include "Doubleslider.h"

SliderWidget::SliderWidget(PCLAnalyzerWindow* mainWnd)
{
  this->mainWnd = mainWnd;

  rminLabel = new QLabel("No Data");
  rmaxLabel = new QLabel("No Data");
  scaleLabel = new QLabel("No Data");
  //sigminLabel = new QLabel("No Data");
  //sigmaxLabel = new QLabel("No Data");

  rminSlider = createRadiusSlider(100);
  rmaxSlider = createRadiusSlider(200);
  scaleSlider = createScaleSlider();
 // sigminSlider = createSigmaSlider();
 // sigmaxSlider = createSigmaSlider();

  connect(rminSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(rminChanged(double)));
  connect(rmaxSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(rmaxChanged(double)));
  connect(scaleSlider, SIGNAL(valueChanged(int)),this,SLOT(scaleChanged(int)));
 // connect(sigminSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(sigminChanged(double)));
 // connect(sigmaxSlider, SIGNAL(doubleValueChanged(double)),this,SLOT(sigmaxChanged(double)));

  connect(rminSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setRmin(double)));
  connect(rmaxSlider, SIGNAL(doubleValueChanged(double)), mainWnd, SLOT(setRmax(double)));
  connect(scaleSlider, SIGNAL(valueChanged(int)), mainWnd, SLOT(setScale(int)));
  //connect(glWidget, SIGNAL(rminChanged(double)), xSlider, SLOT(setValue(int)));
  /*connect(ySlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setYRotation(int)));
  connect(glWidget, SIGNAL(yRotationChanged(int)), ySlider, SLOT(setValue(int)));
  connect(zSlider, SIGNAL(valueChanged(int)), glWidget, SLOT(setZRotation(int)));
  connect(glWidget, SIGNAL(zRotationChanged(int)), zSlider, SLOT(setValue(int)));*/

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

    //mainLayout->addWidget(rminLabel);
    //mainLayout->addWidget(rminSlider);
   // mainLayout->addWidget(rmaxLabel);
   // mainLayout->addWidget(rmaxSlider);
   // mainLayout->addWidget(scaleLabel);
   // mainLayout->addWidget(scaleSlider);


    setLayout(mainLayout);

    rminSlider->setValue(9);
    rmaxSlider->setValue(11);
    scaleSlider->setValue(3);

}

DoubleSlider* SliderWidget::createRadiusSlider(int value)
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

QSlider* SliderWidget::createScaleSlider()
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

void  SliderWidget::rminChanged(double value)
{
    rminLabel->setText("rmin: "+ QString::number(value) );
}

void  SliderWidget::rmaxChanged(double value)
{
    rmaxLabel->setText("rmax: "+ QString::number(value) );
}

void  SliderWidget::scaleChanged(int value)
{
    scaleLabel->setText("scale: "+ QString::number(value) );
}
