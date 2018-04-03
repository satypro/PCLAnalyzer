#include "StructureParameterWidget.h"
#include "DoubleSlider.h"

StructureParameterWidget::StructureParameterWidget(PCLAnalyzerWindow *mainWnd)
{
    this->mainWnd = mainWnd;

    epsLabel = new QLabel("No Data");
    sigminLabel = new QLabel("No Data");
    sigmaxLabel = new QLabel("No Data");
    scalarMinLabel = new QLabel("No Data");
    scalarMaxLabel = new QLabel("No Data");

    epsSlider = CreateEpsillonSlider();
    sigminSlider = CreateSigmaSlider();
    sigmaxSlider = CreateSigmaSlider();
    scalarMaxSlider = CreateEpsillonSlider();
    scalarMinSlider = CreateEpsillonSlider();

    connect(epsSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(EpsChanged(double)));
    connect(sigminSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(SigminChanged(double)));
    connect(sigmaxSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(SigmaxChanged(double)));
    connect(scalarMaxSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(ScalarMaxChanged(double)));
    connect(scalarMinSlider, SIGNAL(DoubleValueChanged(double)),this,SLOT(ScalarMinChanged(double)));

    connect(epsSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetEps(double)));
    connect(sigminSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetSigmaMin(double)));
    connect(sigmaxSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetSigmaMax(double)));
    connect(scalarMaxSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetScalarMax(double)));
    connect(scalarMinSlider, SIGNAL(DoubleValueChanged(double)), mainWnd, SLOT(SetScalarMin(double)));

    QVBoxLayout *mainLayout = new QVBoxLayout;

    QHBoxLayout *hmainLayout = new QHBoxLayout;
    hmainLayout->addWidget(epsSlider);
    hmainLayout->addWidget(epsLabel);

    QHBoxLayout *hmainLayout1 = new QHBoxLayout;
    hmainLayout1->addWidget(sigminSlider);
    hmainLayout1->addWidget(sigminLabel);

    QHBoxLayout *hmainLayout2 = new QHBoxLayout;
    hmainLayout2->addWidget(sigmaxSlider);
    hmainLayout2->addWidget(sigmaxLabel);

    QHBoxLayout *hmainLayout3 = new QHBoxLayout;
    hmainLayout3->addWidget(scalarMinSlider);
    hmainLayout3->addSpacing(20);
    hmainLayout3->addWidget(scalarMinLabel);

    QHBoxLayout *hmainLayout4 = new QHBoxLayout;
    //hmainLayout4->addSpacing(10);
    hmainLayout4->addWidget(scalarMaxSlider);
    hmainLayout4->addWidget(scalarMaxLabel);


    mainLayout->addLayout(hmainLayout);
    mainLayout->addLayout(hmainLayout1);
    mainLayout->addLayout(hmainLayout2);
    mainLayout->addLayout(hmainLayout3);
    mainLayout->addLayout(hmainLayout4);

    setLayout(mainLayout);

    epsSlider->setValue(650);
    sigminSlider->setValue(200);
    sigmaxSlider->setValue(1700);
    scalarMinSlider->setValue(1);
    scalarMaxSlider->setValue(1000);
}

DoubleSlider* StructureParameterWidget::CreateEpsillonSlider()
{
    DoubleSlider *slider = new DoubleSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 1000);
    slider->setSingleStep(100);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

DoubleSlider* StructureParameterWidget::CreateSigmaSlider()
{
    DoubleSlider *slider = new DoubleSlider();
    slider->setOrientation(Qt::Horizontal);
    slider->setRange(0, 2000);
    slider->setSingleStep(1000);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void  StructureParameterWidget::EpsChanged(double value)
{
    epsLabel->setText("Eps : "+ QString::number(value) );
}

void  StructureParameterWidget::SigminChanged(double value)
{
    sigminLabel->setText("Sig Min : "+ QString::number(value) );
}

void  StructureParameterWidget::SigmaxChanged(double value)
{
    sigmaxLabel->setText("Sig Max: "+ QString::number(value) );
}

void  StructureParameterWidget::ScalarMinChanged(double value)
{
    scalarMinLabel->setText("Scalar Min : "+ QString::number(value) );
}

void  StructureParameterWidget::ScalarMaxChanged(double value)
{
    scalarMaxLabel->setText("Scalar Max : "+ QString::number(value) );
}
