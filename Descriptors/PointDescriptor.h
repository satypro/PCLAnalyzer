#ifndef POINTDESCRIPTOR_H
#define POINTDESCRIPTOR_H
#include "Types/datatypes.h"
#include "IPointDescriptor.h"
#include "Classifiers/ClassifierLabels.h"

class PointDescriptor : public IPointDescriptor
{
public:
    probfeatnode featNode;
    featProps    featProp;
    glyphVars    glyph;
    ClassLabels  label;
};

#endif // POINTDESCRIPTOR_H
