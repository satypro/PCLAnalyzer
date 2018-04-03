#include "CovarianceMatrixClassifier.h"
#include "Descriptors/PointDescriptor.h"

CovarianceMatrixClassifier::CovarianceMatrixClassifier()
{
    // Later Configure to load the parameter required from XML file or Json file
    // Currently loading it here by putting the key value;
    _config = new Configuration();
}

Configuration* CovarianceMatrixClassifier::GetConfig()
{
    return _config;
}

IPointDescriptor* CovarianceMatrixClassifier::Classify()
{
    PointDescriptor* pointDescriptor = new PointDescriptor;
    char* pEnd;
    float _epsi = ::strtof(_config->GetValue("epsi").c_str(), &pEnd);
    float _rmin = ::strtof(_config->GetValue("rmin").c_str(), &pEnd);
    float _rmax = ::strtof(_config->GetValue("rmax").c_str(), &pEnd);
    float radius = ::strtof(_config->GetValue("radius").c_str(), &pEnd);

    return pointDescriptor;
}
