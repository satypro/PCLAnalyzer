#ifndef POINTDESCRIPTOR_H
#define POINTDESCRIPTOR_H
#include "Types/datatypes.h"
#include "IPointDescriptor.h"
#include "Classifiers/ClassifierLabels.h"
#include <vector>

class PointDescriptor : public IPointDescriptor
{
public:
    probfeatnode featNode;
    featProps    featProp;
    glyphVars    glyph;
    ClassLabels  label;
    std::vector <unsigned int> PtsProp;
};

#endif // POINTDESCRIPTOR_H
