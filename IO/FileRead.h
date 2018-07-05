#ifndef FILEREAD_H
#define FILEREAD_H
#include "Types/datatypes.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <vector>

class FileRead
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudType;
    FileRead(){}
    ~FileRead(){}

    std::string getFileType(std::string sFilePath);
    bool read(std::string sFilePath, CloudType &cloudData, std::vector <float>&intensity);
    bool readLabels(std::string filename, std::vector <int>&labels);
    void displayLasInfo();

private:
    bool lasFileRead(const std::string sFilePath, CloudType &cloudData, std::vector <float>&intensity);
    bool plyFileRead(const std::string sFilePath,  CloudType &cloudData, std::vector <float>&intensity);
    bool readOriginalPtCloud(std::string filename, CloudType &cloudData, std::vector <float>&intensity);
    bool readLasFileHeaderInfo(FILE *fp);
    bool offFileRead(std::string filename, CloudType &cloudData, std::vector <float>&intensity);
    bool xyzFileRead(std::string filename, CloudType &cloudData, std::vector <float>&intensity);
    SLasHeaderBlock _stPubBlk;
};

#endif // FILEREAD_H
