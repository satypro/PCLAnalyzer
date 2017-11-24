#ifndef DATATYPES_H
#define DATATYPES_H
#include <pcl/common/eigen.h>

struct TensorType
{
    float evec0[3];
    float evec1[3];
    float evec2[3];
};

struct EigenResult
{
    Eigen::Matrix<float, 3 ,1> EigenValues;
    Eigen::Matrix<float, 3, 3> EigenVectors;
};

struct SLasHeaderBlock
{
    unsigned char ucVerMaj;   //1 byte
    unsigned char ucVerMin;  //1 byte
    unsigned char ucFormatID;  //1 byte
    unsigned short usnRecLength;  //2byte
    unsigned int usnNumOfPoint;    //4 byte
    unsigned int usnNumOfPointReturn[5];    //20 byte
    unsigned int ulnOffset;   //4 byte
    double dXScale;
    double dYScale;
    double dZScale;
    double dXOffset;
    double dYOffset;
    double dZOffset;

    double xmax;
    double xmin;

    double ymax;
    double ymin;

    double zmax;
    double zmin;
};

#endif // DATATYPES_H
