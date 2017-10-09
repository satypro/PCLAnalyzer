#ifndef CLOUDPROCESSOR_H
#define CLOUDPROCESSOR_H
#include <iostream>
#include <vector>
#include "result.h"

class CloudProcessor
{
public:
    CloudProcessor();
    std::vector<Result*> getDescriptors(std::string fileName, float radius );
};

#endif // CLOUDPROCESSOR_H
