#ifndef GRAPH_H
#define GRAPH_H
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include "Types/datatypes.h"
#include <pcl/io/pcd_io.h>
#include <Descriptors/PointDescriptor.h>

class Graph
{
public:
    Graph();
    typedef std::vector<idxType> IndxCont;
    typedef std::vector<pcl::PointXYZ> PtCont;
    typedef std::vector<std::vector<idxType>> centroidType;

    void constructGraph(std::vector<gnode> &graph, std::vector<idxType> &graphIdx);
    void setParams(float radius, float rmaxpt);
    void setNode(std::vector<node> &seeds);
    void setCriticalNode(std::vector<node> &crticalNode);
    void setInputcloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setSaliencyMaps(std::vector<PointDescriptor*>& descriptor);
private:

    bool r_search(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts, vector<myfloat3> &maxevecs);
    inline float get_val(myfloat3 u, myfloat3 v, myfloat3 evecs);
    int check(std::vector<gnode>*graph, myfloat3 evecs, int index);
    inline void get_minidx(myfloat3 &u, myfloat3 &v, myfloat3 &evecs, std::vector<tempminval> *tempmin, idxType j);
    bool get_index(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, myfloat3 v, idxType totCount, idxType loc, tempminval *min_idx);
    bool getminWidx(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_sz, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, idxType *min_idx);
    bool propagate(std::vector<node> &seed, std::vector<node> &criticalSeed, idxType minIndex, idxType i, std::vector<gnode> *graph, std::vector<idxType> *graphIdx);
    int connectedge
    (
        std::vector<node> &seed,
        std::vector<node> &criticalSeed,
        IndxCont &neighborSize,
        IndxCont &neighborIndices,
        myfloat3 evecs,
        idxType idx,
        int checkerror,
        std::vector<gnode> *graph,
        std::vector<idxType> *graphIdx,
        idxType *min_idx,
        idxType *min_idx1
    );

    float getDistw(std::vector<node> &seed, std::vector<node> &criticalSeed, myfloat3 v, idxType minIndex);
    bool get_removedindices(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize,
    IndxCont &neighborIndices, idxType index, idxType minIndex, myfloat3 evecs, IndxCont *removedIndices);
    void edgelinking(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize, IndxCont &neighborIndices, std::vector<myfloat3> &maxevecs, std::vector<gnode> *graph, std::vector<idxType> *graphIdx);
    bool r_NearestCriticalNode(std::vector<node> *criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts);
    void r_clusterCriticalNode(std::vector<node> *criticalSeed, centroidType *centroid, float radius);
    idxType get_nearestCentroid(std::vector<idxType> &centroid, idxType idx, std::vector<node> *criticalSeed);
    void connectCentroidalNode(centroidType &centroid, idxType k, idxType i, std::vector<gnode> &graph, std::vector<idxType> &graphIdx);
    bool connectCriticalPoints(IndxCont &ng_sz, IndxCont &ng_idx, std::vector<gnode> &graph, std::vector<idxType> &graphIdx);
    void findDistinctElement(std::vector<idxType> *graphIdx);
    float _radius, _rmaxpt;
    std::vector<node> _node;
    std::vector<node> _critcalNode;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _inCloud;
    std::vector<PointDescriptor*> _descriptor;
};

#endif // GRAPH_H
