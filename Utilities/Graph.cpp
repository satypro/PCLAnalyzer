#include "Graph.h"
#include <GL/gl.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_pointcloud_pointstd::vector.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>

Graph::Graph()
{

}

void Graph::setParams(float radius, float rmaxpt)
{
    _radius = radius;
    _rmaxpt = rmaxpt;
}

void Graph::setNode(std::vector<node> &seeds)
{
    _node = seeds;
}

void Graph::setCriticalNode(std::vector<node> &crticalNode)
{
    _critcalNode = crticalNode;
}

void Graph::setInputcloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    _inCloud = cloud;
}

void Graph::setSaliencyMaps(std::vector<PointDescriptor*>& descriptor)
{
    _descriptor = descriptor;
}


bool Graph::r_search(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts, std::vector<myfloat3> &maxevecs)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr nodeCloud(new pcl::PointCloud<pcl::PointXYZ>);

     nodeCloud->header = _inCloud->header;
     nodeCloud->points.resize(seed.size() + criticalSeed.size());

     for(int m = 0; m < seed.size(); m++)
     {
         int k = seed[m].idx;
         nodeCloud->points[m] = _inCloud->points[k];

     }

     for(int m = seed.size(); m < (seed.size() + criticalSeed.size()) ; m++)
     {
         int k = criticalSeed[m -seed.size()].idx;
         nodeCloud->points[m] = _inCloud->points[k];

     }


    Eigen::Matrix3f eigen_std::vectors;
    Eigen::std::vector3f eigen_values;
    myfloat3 direction;
    Eigen::std::vector4f xyz_centroid;
    Eigen::Matrix3f covariance_matrix;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.005);
    octree.setInputCloud(nodeCloud);
    octree.addPointsFromInputCloud();

    size_t count = 0;

    direction.x = 0.0;
    direction.y = 0.0;
    direction.z = 0.0;


   //edge propagation start from the point having highest feature value
     for(size_t i = 0; i < seed.size(); i++)
     {
       pcl::PointXYZ searchPoint = nodeCloud->points[i];

       octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

       count += pointIdxRadiusSearch.size();

       neighborSize->push_back(count);

       for(size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
       {
           neighborIndices->push_back(pointIdxRadiusSearch[j]);
       }

       if(pointIdxRadiusSearch.size() > 0)
       {
           pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

            tempCloud->header = _inCloud->header;
            tempCloud->points.resize(pointIdxRadiusSearch.size());

            for(int m = 0; m < pointIdxRadiusSearch.size(); m++)
              tempCloud->points[m] = nodeCloud->points[pointIdxRadiusSearch[m]];

           // Estimate the XYZ centroid
             pcl::compute3DCentroid(*tempCloud, xyz_centroid);

             // Compute the 3x3 covariance matrix
             pcl::computeCovarianceMatrix(*tempCloud, xyz_centroid, covariance_matrix);

             // Compute the eigen values of 3x3 covariance matrix
             pcl::eigen33(covariance_matrix, eigen_std::vectors, eigen_values);



            direction.x = eigen_std::vectors(2, 0);
            direction.y = eigen_std::vectors(2, 1);
            direction.z = eigen_std::vectors(2, 2);

            tempCloud->points.clear();

       }


       maxevecs.push_back(direction);

       pointIdxRadiusSearch.clear();
       pointRadiusSquaredDistance.clear();



    }
}

inline float Graph::get_val(myfloat3 u, myfloat3 v, myfloat3 evecs)
{
    myfloat3 diff;

    diff.x = u.x - v.x;
    diff.y = u.y - v.y;
    diff.z = u.z - v.z;
    //normalize

    float invLen = sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

    if(invLen == 0)
        return 0.0;

    diff.x = diff.x / invLen;
    diff.y = diff.y / invLen;
    diff.z = diff.z / invLen;

    float magnitude = evecs.x * diff.x + evecs.y * diff.y + evecs.z * diff.z;

    return magnitude;

}

int Graph::check(std::vector<gnode>*graph, myfloat3 evecs, int index)
{
    if((*graph)[index].edge.size() == 0)
        return 0;

    int i, size = (*graph)[index].edge.size();

    int pos = 0, neg = 0;

    myfloat3 u ,v, evecs1;
    float magnitude, magnitude1;

    evecs1.x = evecs.x * (-1.0);
    evecs1.y = evecs.y * (-1.0);
    evecs1.z = evecs.z * (-1.0);

    int j;

    j = (*graph)[index].nd.idx;

    v.x = _inCloud->points[j].x;
    v.y = _inCloud->points[j].y;
    v.z = _inCloud->points[j].z;

    for(i =0; i < size; i++)
    {
        j = (*graph)[index].edge[i].idx;

        u.x = _inCloud->points[j].x;
        u.y = _inCloud->points[j].y;
        u.z = _inCloud->points[j].z;

        magnitude = get_val(u, v, evecs);

        magnitude1 = get_val(u, v, evecs1);

         if(magnitude > 0)
            pos++;

        if(magnitude1 > 0)
            neg++;

    }

    if(pos > 0 && neg > 0)
        return 1;

    if(pos > 0 && neg <=0)
        return 2;

    if(pos <=0 && neg > 0)
        return 3;

    if(pos == 0 && neg == 0)
        return 4;

    return 5;

}


inline void Graph::get_minidx(myfloat3 &u, myfloat3 &v, myfloat3 &evecs, std::vector<tempminval> *tempmin, idxType j)
{
    myfloat3 diff, w;
    float val, min_val, den;
    tempminval valmin;

    diff.x = u.x - v.x;
    diff.y = u.y - v.y;
    diff.z = u.z - v.z;

    val =  evecs.x * diff.x + evecs.y * diff.y + evecs.z * diff.z;

    w.x =  val *evecs.x + v.x - u.x;
    w.y = val *evecs.y + v.y - u.y;
    w.z = val *evecs.z + v.z - u.z;

    den = sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

    if(den == 0.0)
        min_val = 0.0;
    else
        min_val = (sqrtf(w.x * w.x + w.y * w.y + w.z * w.z))/den ;

    valmin.mval = min_val;
    valmin.idx = j;
    tempmin->push_back(valmin);
}


bool Graph::get_index(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, myfloat3 v, idxType totCount, idxType loc, tempminval *min_idx)
{
    idxType  i, j, m;
    myfloat3 u;
    float magnitude;
    std::vector<tempminval> tempmin(0);

    for( i = 0; i <totCount; i++)
    {
        j =	ng_idxs[i + loc];




        if((j < seed.size() && seed[j].status == false) || j >= seed.size())
        {
            if( j >= seed.size())
                m = crit_seed[j - seed.size()].idx;
            else
                m =  seed[j].idx;

            u.x =  _inCloud->points[m].x;
            u.y =  _inCloud->points[m].y;
            u.z =  _inCloud->points[m].z;

            magnitude = get_val(u, v, evecs);

            if(magnitude > 0.0)
                get_minidx(u, v, evecs, &tempmin, j);

        }

    }

    if(tempmin.size() == 0 || totCount == 0)
        return false;

     float minValue = tempmin[0].mval;
     (*min_idx).mval = minValue;
     (*min_idx).idx =  tempmin[0].idx;

    for(int j =1; j < tempmin.size(); j++)
    {
        if(minValue > tempmin[j].mval)
        {
            (*min_idx).mval = minValue = tempmin[j].mval;
            (*min_idx).idx =  tempmin[j].idx;
        }
    }

    tempmin.clear();

    return true;
}

bool Graph::getminWidx(std::vector<node> &seed, std::vector<node> &crit_seed, IndxCont &ng_sz, IndxCont &ng_idxs, myfloat3 evecs, idxType idx, idxType *min_idx)
{
    idxType totCount, loc;
    myfloat3 v;
    bool  error = false;
    tempminval temp;
    //tempminval temp;

    if(idx == 0)
    {
        totCount = ng_sz[idx];
        loc = 0;
    }
    else
    {
        totCount = ng_sz[idx] - ng_sz[idx-1];
        loc = ng_sz[idx-1];
    }

    v.x =  _inCloud->points[seed[idx].idx].x;
    v.y =  _inCloud->points[seed[idx].idx].y;
    v.z =  _inCloud->points[seed[idx].idx].z;

    *min_idx = -1.0;

    error = get_index(seed, crit_seed, ng_idxs, evecs,  idx, v,  totCount,  loc, &temp);

    if(error == true)
            *min_idx = temp.idx;

    return error;
}

bool Graph::propagate(std::vector<node> &seed, std::vector<node> &criticalSeed, idxType minIndex, idxType i, std::vector<gnode> *graph, std::vector<idxType> *graphIdx)
{
    nodeInfo tempnode;
    idxType m;

    if(minIndex >= seed.size() && minIndex <= (seed.size() + criticalSeed.size()))
    {
        m = criticalSeed[minIndex - seed.size()].idx;

        tempnode.idx = m;
        tempnode.type = 0;

        tempnode.seedIdx = minIndex;

        (*graph)[i].edge.push_back(tempnode);

        graphIdx->push_back(seed[i].idx);
        graphIdx->push_back(m);


    }
    else if (minIndex < seed.size() && minIndex >=0)
    {
        (*graph)[i].edge.push_back((*graph)[minIndex].nd);
        (*graph)[minIndex].edge.push_back((*graph)[i].nd);

        graphIdx->push_back(seed[i].idx);
        graphIdx->push_back(seed[minIndex].idx);

    }
    else
        cout<<"minIndex not in range "<<minIndex<<endl;

}

int Graph::connectedge
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
)
{
    bool error = false, error1 = false;
    idxType temp, temp1;
    myfloat3  d2;
    int type = 0;

    d2.x = -1.0 * evecs.x; 	d2.y = -1.0 * evecs.y;		d2.z = -1.0 * evecs.z;

    *min_idx = *min_idx1 = -1;

    if(checkerror == 0 || checkerror == 4 || checkerror == 5)
    {
        error =  getminWidx(seed, criticalSeed, neighborSize, neighborIndices, evecs, idx, &temp);
        error1 = getminWidx(seed, criticalSeed, neighborSize, neighborIndices, d2, idx, &temp1);
    }
    else if(checkerror == 2)
    {
        error1 = getminWidx(seed, criticalSeed, neighborSize, neighborIndices, d2, idx, &temp1);
    }
    else if(checkerror == 3)
    {
        error = getminWidx(seed, criticalSeed, neighborSize, neighborIndices, evecs, idx, &temp);
    }
    else
    {
        cout<<"wrong check parameter"<<endl;
        return 0;
    }

    if(error == false && error1 == false)
        return 0;

    if(error == true )
    {
        if(temp < 0)
            cout<<"temp "<<temp<<endl;
        else
        {
            propagate(seed, criticalSeed, temp, idx, graph, graphIdx);
            *min_idx = temp; //.idx;
        }
    }


    if(error1 == true)
    {
        //cout<<"me "<<endl;
        if(temp1 <0)
            cout<<"temp1 "<<temp1<<endl;
        else
        {
            propagate(seed, criticalSeed, temp1, idx, graph, graphIdx);
            *min_idx1 = temp1; //.idx;
        }
    }


    if(error == true && error1 == true)
        type = 1;
    else if(error == true && error1 == false)
        type = 2;
    else if(error == false && error1 == true)
        type = 3;
    else
        type = 0;


    return type;
}

float Graph::getDistw(std::vector<node> &seed, std::vector<node> &criticalSeed, myfloat3 v, idxType minIndex)
{
    idxType m;
    float dist;
    myfloat3 w;

    if( minIndex >= seed.size())
        m = criticalSeed[minIndex - seed.size()].idx;
    else
        m =  seed[minIndex].idx;

    w.x =  _inCloud->points[m].x;
    w.y =  _inCloud->points[m].y;
    w.z =  _inCloud->points[m].z;

    dist = sqrt(pow((v.x - w.x), 2.0) + pow((v.y - w.y), 2.0) + pow((v.z - w.z), 2.0));

    return dist;
}

bool Graph::get_removedindices(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize,
IndxCont &neighborIndices, idxType index, idxType minIndex, myfloat3 evecs, IndxCont *removedIndices)
{
    idxType totCount, loc, m,i ,j;
    myfloat3 u, v;
    float dist_w = -1.0,  dist, ang;

    if(index == 0)
    {
        totCount = neighborSize[index];
        loc = 0;
    }
    else
    {
        totCount = neighborSize[index] - neighborSize[index-1];
        loc = neighborSize[index-1];
    }

    if(totCount == 0)
        return false;

    v.x =  _inCloud->points[seed[index].idx].x;
    v.y =  _inCloud->points[seed[index].idx].y;
    v.z =  _inCloud->points[seed[index].idx].z;

    //cout<<"minIndex "<<minIndex<<endl;
    //cout<<" minIndex1 "<<minIndex1<<endl;

    if(minIndex >= 0)
        dist_w = getDistw(seed, criticalSeed, v, minIndex);

    for(i = 0; i < totCount; i++)
    {
        j =	neighborIndices[i + loc];

        if( j >= seed.size())
            m = criticalSeed[j - seed.size()].idx;
        else
            m =  seed[j].idx;

        u.x =  _inCloud->points[m].x;
        u.y =  _inCloud->points[m].y;
        u.z =  _inCloud->points[m].z;

        dist = sqrt(pow((v.x - u.x), 2.0) + pow((v.y - u.y), 2.0) + pow((v.z - u.z), 2.0));

        ang = get_val(u, v, evecs);

        if(dist < dist_w && j < seed.size() && ang > 0.0)
            removedIndices->push_back(j);
    }
    return true;
}

void Graph::edgelinking(std::vector<node> &seed, std::vector<node> &criticalSeed, IndxCont &neighborSize, IndxCont &neighborIndices, std::vector<myfloat3> &maxevecs, std::vector<gnode> *graph, std::vector<idxType> *graphIdx)
{
    idxType i, minIndex, minIndex1, checkerror;
    myfloat3 evecs;
    bool  id_st, remo_st;
    IndxCont removedIndices;

    int count = 0;

    for(i = 0; i < seed.size(); i++)
    {
        evecs = maxevecs[i];
        checkerror =  check(graph, evecs, i);

        id_st = 0;
        remo_st = false;

        if(seed[i].status == false && checkerror != 1)
        {
            //cout<<" i am here "<<endl;
            id_st  = connectedge(seed, criticalSeed, neighborSize, neighborIndices, evecs, i, checkerror, graph, graphIdx, &minIndex, &minIndex1);

            if(id_st == 1 || id_st == 2)
            {
                remo_st = get_removedindices(seed, criticalSeed, neighborSize, neighborIndices, i,
                     minIndex, evecs, &removedIndices);
                if(remo_st == true && removedIndices.size() > 0)
                {
                    for(int k =0; k <removedIndices.size(); k++)
                    {
                        if((*graph)[removedIndices[k]].edge.size() == 0)
                            seed[removedIndices[k]].status = true;
                    }

                    count += removedIndices.size();
                }
                removedIndices.clear();

            }
            if(id_st == 1 || id_st == 3)
            {
                //cout<<"t "<<endl;
                myfloat3 evecs1;
                evecs1.x = (-1.0) *evecs.x;
                evecs1.y = (-1.0) *evecs.y;
                evecs1.z = (-1.0) *evecs.z;

                remo_st = get_removedindices(seed, criticalSeed, neighborSize, neighborIndices, i,
                     minIndex1, evecs1, &removedIndices);

                if(remo_st == true && removedIndices.size() > 0)
                {
                    for(int k =0; k <removedIndices.size(); k++)
                    {
                        if((*graph)[removedIndices[k]].edge.size() == 0)
                            seed[removedIndices[k]].status = true;
                    }

                    count += removedIndices.size();
                }
                removedIndices.clear();
            }

            if(id_st == 0)
            {
                graphIdx->push_back(seed[i].idx);
            }



        }


    }

    return;
}

bool Graph::r_NearestCriticalNode(std::vector<node> *criticalSeed, IndxCont *neighborIndices, IndxCont *neighborSize, ftType radius, idxType rmaxpts)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr nodeCloud(new pcl::PointCloud<pcl::PointXYZ>);

     nodeCloud->header = _inCloud->header;
     nodeCloud->points.resize(criticalSeed->size());

     for(int m = 0; m < criticalSeed->size(); m++)
     {
         int k = (*criticalSeed)[m].idx;
         nodeCloud->points[m] = _inCloud->points[k];

     }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.005);
    octree.setInputCloud(nodeCloud);
    octree.addPointsFromInputCloud();

    octree.setInputCloud(nodeCloud);
    octree.addPointsFromInputCloud();

    size_t count = 0;

   //edge propagation start from the point having highest feature value
     for(size_t i = 0; i < nodeCloud->points.size(); i++)
     {
       pcl::PointXYZ searchPoint = nodeCloud->points[i];

       octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

       count += pointIdxRadiusSearch.size();

       neighborSize->push_back(count);

       for(size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
       {
           neighborIndices->push_back(pointIdxRadiusSearch[j]);
       }

       pointIdxRadiusSearch.clear();
       pointRadiusSquaredDistance.clear();

    }

    return true;

}

void Graph::r_clusterCriticalNode(std::vector<node> *criticalSeed, centroidType *centroid, float radius)
{
    IndxCont neighborIndices, neighborSize;

    r_NearestCriticalNode(criticalSeed, &neighborIndices, &neighborSize, radius, _rmaxpt);

    idxType totCount, loc, i, j, k;

    std::vector< idxType > tempidx, tempcent;

    double weight, max_wt;

    centroid->resize(criticalSeed->size());
    //cout<<"critical seed size "<<criticalSeed->size()<<endl;
    for(i = 0; i < criticalSeed->size(); i++)
    {
        if((*criticalSeed)[i].status == 0)
        {
            tempidx.push_back(i);  // contains cluster indices
            max_wt = _descriptor[(*criticalSeed)[i].idx]->featNode.csclcp[0];

            if(i == 0)
            {
                totCount = neighborSize[i];
                loc = 0;
            }
            else
            {
                totCount = neighborSize[i] - neighborSize[i-1];
                loc = neighborSize[i-1];
            }

            for(j = 0; j < totCount; j++)
            {
                k = neighborIndices[j + loc];

                if( (*criticalSeed)[k].status == false && k!=i)
                    tempidx.push_back(k);
            }

            for(j = 0; j < tempidx.size(); j++)
            {
                weight = _descriptor[(*criticalSeed)[tempidx[j]].idx]->featNode.csclcp[0];
                if(weight > max_wt)
                    max_wt = weight;
            }

            for(j = 0; j < tempidx.size(); j++)
            {
                weight = _descriptor[(*criticalSeed)[tempidx[j]].idx]->featNode.csclcp[0];
                if(max_wt == weight)
                    tempcent.push_back(tempidx[j]);
            }

            for(j = 0; j < tempidx.size(); j++)
            {

                for(k = 0; k < tempcent.size(); k++)
                    (*centroid)[tempidx[j]].push_back(tempcent[k]);
                (*criticalSeed)[tempidx[j]].status = true;
            }

            tempcent.clear();
            tempidx.clear();
        }
    }


}

idxType Graph::get_nearestCentroid(std::vector<idxType> &centroid, idxType idx, std::vector<node> *criticalSeed)
{
    idxType i, j , m, centroidIdx = 0;
    float min_dist, dist;

    centroidIdx = j = centroid[0];
    m = (*criticalSeed)[j].idx;

    pcl::PointXYZ pt1 = _inCloud->points[idx];
    pcl::PointXYZ pt2 = _inCloud->points[m];

    min_dist = sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0) + pow(pt1.z - pt2.z, 2.0)) ;

    for(i = 1; i < centroid.size(); i++)
    {
        j = centroid[i];

        m = (*criticalSeed)[j].idx;

        pt2 = _inCloud->points[m];

        dist = sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0) + pow(pt1.z - pt2.z, 2.0));

        if(dist  < min_dist)
            {
            min_dist = dist;
            centroidIdx = j;
            }
    }

    return centroidIdx;
}

void Graph::connectCentroidalNode(centroidType &centroid, idxType k, idxType i, std::vector<gnode> &graph, std::vector<idxType> &graphIdx)
{
    idxType centroid_idx, temp;
    nodeInfo tempnode;
    //centroid size is zero means it is only centroidal node otherwise find the nearest centroidal node

    if( centroid[k].size() == 0)
    {
        centroid_idx = _critcalNode[k].idx;

        tempnode.idx = centroid_idx;
        tempnode.type = 0;
        tempnode.seedIdx = k + _node.size();

        graph[i].edge.push_back(tempnode);
        graphIdx.push_back(_node[i].idx);
        graphIdx.push_back(centroid_idx);
    }
    else if(centroid[k].size() == 1)
    {
        temp = centroid[k][0];
        centroid_idx = _critcalNode[temp].idx;

        tempnode.idx = centroid_idx;
        tempnode.type = 0;
        tempnode.seedIdx = k + _node.size();


        graph[i].edge.push_back(tempnode);

        graphIdx.push_back(_node[i].idx);
        graphIdx.push_back(centroid_idx);
    }
    else if (centroid[k].size() > 1)
    {
        temp = get_nearestCentroid(centroid[k], _node[i].idx, &_critcalNode);
        centroid_idx = _critcalNode[temp].idx;

        tempnode.idx = centroid_idx;
        tempnode.type = 0;
        tempnode.seedIdx = k + _node.size();


        graph[i].edge.push_back(tempnode);

        graphIdx.push_back(_node[i].idx);
        graphIdx.push_back(centroid_idx);
    }

}

bool Graph::connectCriticalPoints(IndxCont &ng_sz, IndxCont &ng_idx, std::vector<gnode> &graph, std::vector<idxType> &graphIdx)
{
    if(_critcalNode.size() == 0)
    {
        cout<<"no critical points"<<endl;
        return false;
    }

    //radius based clustering of critical points and also find centroidal node
    centroidType centroid;

    r_clusterCriticalNode(&_critcalNode, &centroid, _radius);

    //connect critical node
    idxType i, j, k, totCount, loc;

    //cout<<" seed.size()"<<seed.size()<<endl;
    for(i = 0; i < _node.size(); i++)
    {
        if(i == 0)
        {
            totCount = ng_sz[i];
            loc = 0;
        }
        else
        {
            totCount = ng_sz[i] - ng_sz[i-1];
            loc = ng_sz[i-1];
        }

        for( j = 0; j < totCount; j++)
        {

            k = ng_idx[j + loc];

            if(k >= _node.size())
            {
                k = k - _node.size();

                connectCentroidalNode(centroid, k, i, graph, graphIdx);
            }
        }
    }

    return true;

}

void Graph::findDistinctElement(std::vector<idxType> *graphIdx)
{
    std::vector<idxType> tempa(graphIdx->size());
    std::vector<idxType>::iterator it;

    std::copy(graphIdx->begin(), graphIdx->end(), tempa.begin());

    std::sort(tempa.begin(), tempa.end());

    it = std::unique (tempa.begin(), tempa.end());
    tempa.resize( std::distance(tempa.begin(),it) );

    graphIdx->resize(tempa.size());

    for(int i = 0; i < tempa.size(); i++)
        (*graphIdx)[i] = tempa[i];

   tempa.clear();

}

void Graph::constructGraph(std::vector<gnode> &graph, std::vector<idxType> &graphIdx)
{
    IndxCont neighborIndices, neighborSize;
    if(_node.size() == 0)
    {
        cout<<"Initialseed size is zero. Can't generate any graph"<<endl;
        return;
    }

    std::vector<myfloat3> maxevecs;

    r_search(_node, _critcalNode, &neighborIndices, &neighborSize, _radius, _rmaxpt, maxevecs);

    size_t i, m;

    nodeInfo tempnode;

    cout<<"size  maxevecs "<<maxevecs.size()<<endl;

    graph.resize(_node.size());

    for(i = 0; i < _node.size(); i++)
    {
        m = _node[i].idx;

        tempnode.idx = m;
        tempnode.type = true;
        tempnode.seedIdx = i;
        graph[i].nd = tempnode;
    }

    edgelinking(_node, _critcalNode, neighborSize, neighborIndices, maxevecs, &graph, &graphIdx);
    connectCriticalPoints(neighborSize, neighborIndices, graph, graphIdx);

    findDistinctElement(&graphIdx);
}
