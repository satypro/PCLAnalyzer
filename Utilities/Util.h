#ifndef UTIL_H
#define UTIL_H
#include "Types/datatypes.h"
#include "Neighbours/SearchNeighbourBase.h"
#include "Descriptors/PointDescriptor.h"

class Util
{
public:
    Util();
    pcl::PointCloud<pcl::PointXYZ>::Ptr _inCloud;

    void SetSearchStrategy(SearchNeighbourBase* searchNeighbour)
    {
        _searchNeighbour = searchNeighbour;
    }

    SearchNeighbourBase*  GetSearchStrategy()
    {
            return _searchNeighbour;
    }

    void SetParams(float rmin, float rmax, float rmaxpt, float epsi, float scale)
    {
        _rmin = rmin;
        _rmax = rmax;
        _rmaxpt = rmaxpt;
        _epsi = epsi;
        _scale = scale;
    }

    void ComputeDoNs_MSO(std::vector<PointDescriptor*>& descriptor,float rmin, float rmax);
    void Triangulate(float radius, std::vector<PointDescriptor*>& descriptor);
    void GenerateTensorLines3(std::vector<PointDescriptor*>& descriptor);
    void PruneTriangles(float radius, std::vector<PointDescriptor*>& descriptor);

private:
    SearchNeighbourBase* _searchNeighbour;
    float _rmin, _rmax, _rmaxpt, _epsi, _scale;
    float max_cl, min_cl;
    int numTriangles;
};

#endif // UTIL_H
