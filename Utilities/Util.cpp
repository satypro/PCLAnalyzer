#include "Util.h"
#include <algorithm>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_euclidean_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Segment_3.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <teem/ten.h>
#include "Utilities/eig3.h"


#define GROUND_THRESHOLD 0.0132672
#define CONTOUR_VALUE 0.4
#define DOUGLASPEUCKER_EPSILON 0.005
#define NUM_SEED_POINTS 3500
#define SEED_POINTS_INTERVAL 80
#define STEP_SIZE 0.003
#define MAX_LENGTH 100
#define DON_LOW 0.3
#define DON_HIGH 0.9

struct K : CGAL::Exact_predicates_inexact_constructions_kernel {};

typedef CGAL::Triangulation_euclidean_traits_xy_3<K>  Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;

typedef K::Point_3   Point;
typedef K::Triangle_3  Triangle_3;
typedef K::Vector_3  Vector_3;
typedef K::Segment_3  Segment_3;

Util::Util()
{

}

void Util::ComputeDoNs_MSO(std::vector<PointDescriptor*>& descriptor,float rmin, float rmax)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud <pcl::Normal>::Ptr normals1 (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator1;

    pcl::PointCloud <pcl::Normal>::Ptr normals2 (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator2;

    normal_estimator1.setSearchMethod (tree);
    normal_estimator1.setInputCloud (_inCloud);
    normal_estimator1.setRadiusSearch (rmin);
    normal_estimator1.compute (*normals1);

    normal_estimator2.setSearchMethod (tree);
    normal_estimator2.setInputCloud (_inCloud);
    normal_estimator2.setRadiusSearch (rmax);
    normal_estimator2.compute (*normals2);

    float max = -1.0;
    float min = 10000.0;

    for(size_t i =0; i < _inCloud->points.size(); i++)
    {

        float n1 = sqrt(((*normals1)[i].normal_x-(*normals2)[i].normal_x) * ((*normals1)[i].normal_x-(*normals2)[i].normal_x) +
                        ((*normals1)[i].normal_y-(*normals2)[i].normal_y) * ((*normals1)[i].normal_y-(*normals2)[i].normal_x) +
                        ((*normals1)[i].normal_z-(*normals2)[i].normal_z) * ((*normals1)[i].normal_z-(*normals2)[i].normal_x) );

        descriptor[i]->featNode.don = (n1)/2;

        if(descriptor[i]->featNode.don > max)
            max = descriptor[i]->featNode.don;
        if(descriptor[i]->featNode.don < min)
            min = descriptor[i]->featNode.don;
    }

    for(size_t i =0; i < _inCloud->points.size(); i++)
    {
        descriptor[i]->featNode.don = (descriptor[i]->featNode.don - min ) / (max - min);
    }
}

void Util::Triangulate(float radius, std::vector<PointDescriptor*>& descriptor)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ searchPoint, tempPoint;

    max_cl = -1;
    min_cl = 100;

    numTriangles = 0;
    int countDONlines = 0;

    float don_low = DON_LOW;
    float don_high = DON_HIGH;


        for(size_t i =0; i < _inCloud->points.size(); i++)
        {
            searchPoint = _inCloud->points[i];
            std::vector<Point> triangle_points;
            std::vector<int> triangle_points_ndx;

            if(descriptor[i]->featNode.don>=don_low && descriptor[i]->featNode.don<=don_high)
                countDONlines++;
            bool triPoints = (descriptor[i]->featNode.csclcp[1]>=.25 && descriptor[i]->featNode.csclcp[1]<=0.55) || (descriptor[i]->featNode.don>=don_low && descriptor[i]->featNode.don<=don_high) || (descriptor[i]->featNode.csclcp[1] > descriptor[i]->featNode.csclcp[0] && descriptor[i]->featNode.csclcp[1] > descriptor[i]->featNode.csclcp[2]);
            if( triPoints )
            {
                if(max_cl<descriptor[i]->featNode.csclcp[1])
                    max_cl = descriptor[i]->featNode.csclcp[1];
                if(min_cl>descriptor[i]->featNode.csclcp[1])
                    min_cl = descriptor[i]->featNode.csclcp[1];

                _searchNeighbour->searchOption.searchParameter.radius = radius;
                pcl::PointCloud<pcl::PointXYZ>::Ptr _neighbourCloud =
                        _searchNeighbour->GetNeighbourCloud(searchPoint);
                pointIdxRadiusSearch = _searchNeighbour->GetNeighbourCloudIndex();

                for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++)
                {
                    if((descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[1]>=.25
                        && descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[1]<=.55 )
                            || (descriptor[pointIdxRadiusSearch[j]]->featNode.don>=don_low
                                && descriptor[pointIdxRadiusSearch[j]]->featNode<=don_high)
                            || (descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[1] > descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[0]
                                && descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[1] > descriptor[pointIdxRadiusSearch[j]]->featNode.csclcp[2])
                    )
                    {
                        tempPoint = _inCloud->points[pointIdxRadiusSearch[j]];
                        triangle_points.push_back(Point(tempPoint.x,tempPoint.y,tempPoint.z));
                        triangle_points_ndx.push_back(pointIdxRadiusSearch[j]);
                    }
                }

                triangle_points.push_back(Point(searchPoint.x,searchPoint.y,searchPoint.z));
                triangle_points_ndx.push_back(i);
                Delaunay dt;
                dt.insert(triangle_points.begin(), triangle_points.end());
                std::vector<Triangle> triangles;
                for(Delaunay::Finite_faces_iterator it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it)
                {
                    Point p1 = it->vertex(0)->point();
                    Point p2 = it->vertex(1)->point();
                    Point p3 = it->vertex(2)->point();
                    int i1=-1;
                    int i2=-1;
                    int i3=-1;
                    for(int k=0;k<triangle_points.size();k++)
                    {
                        if(p1==triangle_points[k]){
                            i1 = triangle_points_ndx[k];
                        }
                        if(p2==triangle_points[k]){
                            i2 = triangle_points_ndx[k];
                        }
                        if(p3==triangle_points[k]){
                            i3 = triangle_points_ndx[k];
                        }
                    }

                    Triangle temp;
                    temp.p[0] = p1.x();
                    temp.p[1] = p1.y();
                    temp.p[2] = p1.z();
                    temp.q[0] = p2.x();
                    temp.q[1] = p2.y();
                    temp.q[2] = p2.z();
                    temp.r[0] = p3.x();
                    temp.r[1] = p3.y();
                    temp.r[2] = p3.z();
                    temp.pid  = i1;
                    temp.qid  = i2;
                    temp.rid  = i3;
                    temp.average_cl = (descriptor[i1]->featNode.csclcp[1] + descriptor[i2]->featNode.csclcp[1] + descriptor[i3]->featNode.csclcp[1])/3.0;
                    temp.average_anisotropy = (descriptor[i1]->featNode.anisotropy + descriptor[i2]->featNode.anisotropy + descriptor[i3]->featNode.anisotropy)/3.0;

                    Vector_3 pq(p1,p2);
                    Vector_3 qr(p2,p3);
                    Vector_3 rp(p3,p1);
                    pq = pq/sqrt(pq.squared_length());
                    qr = qr/sqrt(qr.squared_length());
                    rp = rp/sqrt(rp.squared_length());

                    float score_pq = abs(pq.x()*descriptor[i1]->featNode.evecs[0] + pq.y()*descriptor[i1]->featNode.evecs[1] + pq.z()*descriptor[i1]->featNode.evecs[2]);
                    float score_pr = abs(rp.x()*descriptor[i1]->featNode.evecs[0] + rp.y()*descriptor[i1]->featNode.evecs[1] + rp.z()*descriptor[i1]->featNode.evecs[2]);

                    float score_qp = abs(pq.x()*descriptor[i2]->featNode.evecs[0] + pq.y()*descriptor[i2]->featNode.evecs[1] + pq.z()*descriptor[i2]->featNode.evecs[2]);
                    float score_qr = abs(qr.x()*descriptor[i2]->featNode.evecs[0] + qr.y()*descriptor[i2]->featNode.evecs[1] + qr.z()*descriptor[i2]->featNode.evecs[2]);

                    float score_rp = abs(rp.x()*descriptor[i1]->featNode.evecs[0] + rp.y()*descriptor[i2]->featNode.evecs[1] + rp.z()*descriptor[i3]->featNode.evecs[2]);
                    float score_rq = abs(qr.x()*descriptor[i1]->featNode.evecs[0] + qr.y()*descriptor[i2]->featNode.evecs[1] + qr.z()*descriptor[i3]->featNode.evecs[2]);

                    temp.tlScore = max(score_pq,max(score_pr,max(score_qp,max(score_qr,max(score_rp,score_rq)))));

                    if(temp.tlScore == score_pq || temp.tlScore == score_qp)
                        temp.tlEdge = 0;
                    else if(temp.tlScore == score_qr || temp.tlScore == score_rq)
                        temp.tlEdge = 1;
                    else
                        temp.tlEdge = 2;

                    float average_height_tri = (temp.p[2] + temp.q[2] + temp.r[2])/3;

                    // Marching Triangles
                    float scalar1 = descriptor[i1]->featNode.csclcp[1];
                    float scalar2 = descriptor[i2]->featNode.csclcp[1];
                    float scalar3 = descriptor[i3]->featNode.csclcp[1];

                    if(scalar1 > CONTOUR_VALUE && scalar2 > CONTOUR_VALUE && scalar3 > CONTOUR_VALUE ) {
                        //+++
                        temp.hasCL = false;
                    } else if(scalar1 > CONTOUR_VALUE && scalar2 > CONTOUR_VALUE && scalar3 <= CONTOUR_VALUE) {
                        //++-
                        temp.hasCL = true;
                        float per1 = (scalar1 - CONTOUR_VALUE) / (scalar1 - scalar3);
                        float per2 = (scalar2 - CONTOUR_VALUE) / (scalar2 - scalar3);

                        temp.CL_p1[0] = p1.x()*(1-per1) + p3.x()*per1;
                        temp.CL_p1[1] = p1.y()*(1-per1) + p3.y()*per1;
                        temp.CL_p1[2] = p1.z()*(1-per1) + p3.z()*per1;

                        temp.CL_p2[0] = p2.x()*(1-per2) + p3.x()*per2;
                        temp.CL_p2[1] = p2.y()*(1-per2) + p3.y()*per2;
                        temp.CL_p2[2] = p2.z()*(1-per2) + p3.z()*per2;

                    } else if(scalar1 > CONTOUR_VALUE && scalar2 <= CONTOUR_VALUE && scalar3 > CONTOUR_VALUE) {
                        //+-+
                        temp.hasCL = true;
                        float per1 = (scalar1 - CONTOUR_VALUE) / (scalar1 - scalar2);
                        float per2 = (scalar3 - CONTOUR_VALUE) / (scalar3 - scalar2);

                        temp.CL_p1[0] = p1.x()*(1-per1) + p2.x()*per1;
                        temp.CL_p1[1] = p1.y()*(1-per1) + p2.y()*per1;
                        temp.CL_p1[2] = p1.z()*(1-per1) + p2.z()*per1;

                        temp.CL_p2[0] = p3.x()*(1-per2) + p2.x()*per2;
                        temp.CL_p2[1] = p3.y()*(1-per2) + p2.y()*per2;
                        temp.CL_p2[2] = p3.z()*(1-per2) + p2.z()*per2;

                    } else if(scalar1 > CONTOUR_VALUE && scalar2 <= CONTOUR_VALUE && scalar3 <= CONTOUR_VALUE) {
                        //+--
                        temp.hasCL = true;
                        float per1 = (scalar1 - CONTOUR_VALUE) / (scalar1 - scalar2);
                        float per2 = (scalar1 - CONTOUR_VALUE) / (scalar1 - scalar3);

                        temp.CL_p1[0] = p1.x()*(1-per1) + p2.x()*per1;
                        temp.CL_p1[1] = p1.y()*(1-per1) + p2.y()*per1;
                        temp.CL_p1[2] = p1.z()*(1-per1) + p2.z()*per1;

                        temp.CL_p2[0] = p1.x()*(1-per2) + p3.x()*per2;
                        temp.CL_p2[1] = p1.y()*(1-per2) + p3.y()*per2;
                        temp.CL_p2[2] = p1.z()*(1-per2) + p3.z()*per2;

                    } else if(scalar1 <= CONTOUR_VALUE && scalar2 > CONTOUR_VALUE && scalar3 > CONTOUR_VALUE) {
                        //-++
                        temp.hasCL = true;
                        float per1 = (scalar2 - CONTOUR_VALUE) / (scalar2 - scalar1);
                        float per2 = (scalar3 - CONTOUR_VALUE) / (scalar3 - scalar1);

                        temp.CL_p1[0] = p2.x()*(1-per1) + p1.x()*per1;
                        temp.CL_p1[1] = p2.y()*(1-per1) + p1.y()*per1;
                        temp.CL_p1[2] = p2.z()*(1-per1) + p1.z()*per1;

                        temp.CL_p2[0] = p3.x()*(1-per2) + p1.x()*per2;
                        temp.CL_p2[1] = p3.y()*(1-per2) + p1.y()*per2;
                        temp.CL_p2[2] = p3.z()*(1-per2) + p1.z()*per2;

                    } else if(scalar1 <= CONTOUR_VALUE && scalar2 > CONTOUR_VALUE && scalar3 <= CONTOUR_VALUE) {
                        //-+-
                        temp.hasCL = true;
                        float per1 = (scalar2 - CONTOUR_VALUE) / (scalar2 - scalar1);
                        float per2 = (scalar2 - CONTOUR_VALUE) / (scalar2 - scalar3);

                        temp.CL_p1[0] = p2.x()*(1-per1) + p1.x()*per1;
                        temp.CL_p1[1] = p2.y()*(1-per1) + p1.y()*per1;
                        temp.CL_p1[2] = p2.z()*(1-per1) + p1.z()*per1;

                        temp.CL_p2[0] = p2.x()*(1-per2) + p3.x()*per2;
                        temp.CL_p2[1] = p2.y()*(1-per2) + p3.y()*per2;
                        temp.CL_p2[2] = p2.z()*(1-per2) + p3.z()*per2;
                    } else if(scalar1 <= CONTOUR_VALUE && scalar2 <= CONTOUR_VALUE && scalar3 > CONTOUR_VALUE) {
                        //--+
                        temp.hasCL = true;
                        float per1 = (scalar3 - CONTOUR_VALUE) / (scalar3 - scalar1);
                        float per2 = (scalar3 - CONTOUR_VALUE) / (scalar3 - scalar2);

                        temp.CL_p1[0] = p3.x()*(1-per1) + p1.x()*per1;
                        temp.CL_p1[1] = p3.y()*(1-per1) + p1.y()*per1;
                        temp.CL_p1[2] = p3.z()*(1-per1) + p1.z()*per1;

                        temp.CL_p2[0] = p3.x()*(1-per2) + p2.x()*per2;
                        temp.CL_p2[1] = p3.y()*(1-per2) + p2.y()*per2;
                        temp.CL_p2[2] = p3.z()*(1-per2) + p2.z()*per2;
                    } else if(scalar1 <= CONTOUR_VALUE && scalar2 <= CONTOUR_VALUE && scalar3 <= CONTOUR_VALUE) {
                        //---
                        temp.hasCL = false;
                    }

                    if(average_height_tri > 0.02)
                        triangles.push_back(temp);
                }
                descriptor[i]->featNode.triangles = triangles;
                numTriangles += triangles.size();

                pointIdxRadiusSearch.clear();
            }
        }
        std::cout << "count dons = " << countDONlines << std::endl;
}

void Util::GenerateTensorLines3(std::vector<PointDescriptor*>& descriptor)
{
    // Get some random seed point
    std::vector<int> seed_points;
    for(size_t j =0; j < _inCloud->points.size(); j++)
    {
        if(descriptor[j]->featNode.triangles.size()>0){
            seed_points.push_back(j);
            j+= SEED_POINTS_INTERVAL;
            if(seed_points.size()>=NUM_SEED_POINTS)
                break;
        }
    }
    std::cout << "# seed points = " << seed_points.size() << std::endl;
    for(int sp = 0; sp<seed_points.size(); sp++)
    {
        int seed_point = seed_points[sp];
        descriptor[seed_point]->featNode.tl_visited = true;

        float step_size = STEP_SIZE;
        float max_length = MAX_LENGTH;
        float current_length = 0.0;

        std::vector<Line> tensor_line;

        float current_point[3] = {
            _inCloud->points[seed_point].x,
            _inCloud->points[seed_point].y,
            _inCloud->points[seed_point].z
        }; //=seed point

        float next_point[3] = {0,0,0};

        float major_evec[3] = {
            descriptor[seed_point]->featNode.evecs[0],
            descriptor[seed_point]->featNode.evecs[1],
            descriptor[seed_point]->featNode.evecs[2]
        };

        std::vector<Triangle> neighbouring_triangles;
        neighbouring_triangles.insert(neighbouring_triangles.end(),
                                      descriptor[seed_point]->featNode.triangles.begin(),
                                      descriptor[seed_point]->featNode.triangles.end());

        while(current_length<max_length)
        {
            next_point[0] = step_size * major_evec[0] + current_point[0];
            next_point[1] = step_size * major_evec[1] + current_point[1];
            next_point[2] = step_size * major_evec[2] + current_point[2];

            float minDis = 1000;
            float np_id = -1;
            Point cp(current_point[0],current_point[1],current_point[2]);
            Point np(next_point[0],next_point[1],next_point[2]);
            for(int i=0;i<neighbouring_triangles.size();i++)
            {
                Point p(neighbouring_triangles[i].p[0], neighbouring_triangles[i].p[1], neighbouring_triangles[i].p[2]);
                Point q(neighbouring_triangles[i].q[0], neighbouring_triangles[i].q[1], neighbouring_triangles[i].q[2]);
                Point r(neighbouring_triangles[i].r[0], neighbouring_triangles[i].r[1], neighbouring_triangles[i].r[2]);
                Segment_3 cpp(np,p);
                Segment_3 cpq(np,q);
                Segment_3 cpr(np,r);
                if(!descriptor[neighbouring_triangles[i].pid]->featNode.tl_visited && p!=cp && cpp.squared_length()<minDis) {
                    minDis = cpp.squared_length();
                    np_id = neighbouring_triangles[i].pid;
                    descriptor[np_id]->featNode.tl_visited = true;
                }
                if(!descriptor[neighbouring_triangles[i].qid]->featNode.tl_visited && q!=cp && cpq.squared_length()<minDis) {
                    minDis = cpq.squared_length();
                    np_id = neighbouring_triangles[i].qid;
                    descriptor[np_id]->featNode.tl_visited = true;
                }
                if(!descriptor[neighbouring_triangles[i].rid]->featNode.tl_visited && r!=cp && cpr.squared_length()<minDis) {
                    minDis = cpr.squared_length();
                    np_id = neighbouring_triangles[i].rid;
                    descriptor[np_id]->featNode.tl_visited = true;
                }
            }
            if(np_id==-1)
                break;
            else {
                Line step;
                step.p[0] = current_point[0];
                step.p[1] = current_point[1];
                step.p[2] = current_point[2];
                step.q[0] = _inCloud->points[np_id].x;
                step.q[1] = _inCloud->points[np_id].y;
                step.q[2] = _inCloud->points[np_id].z;
                step.ndxq = np_id;
                tensor_line.push_back(step);
                current_length += step_size;
                current_point[0] = _inCloud->points[np_id].x;
                current_point[1] = _inCloud->points[np_id].y;
                current_point[2] = _inCloud->points[np_id].z;
                neighbouring_triangles.clear();
                neighbouring_triangles.insert(neighbouring_triangles.end(),
                                              descriptor[np_id]->featNode.triangles.begin(),
                                              descriptor[np_id]->featNode.triangles.end());
            }
        }
        descriptor[seed_point]->featNode.tensor_line = tensor_line;
    }
}

void Util::PruneTriangles(float radius, std::vector<PointDescriptor*>& descriptor)
{
    float threshold = (max_cl+min_cl)*.25;
    numTriangles = 0;

    for(size_t i =0; i < _inCloud->points.size(); i++)
    {
        for (std::vector<Triangle>::iterator it=(descriptor[i]->featNode.triangles).begin();
                                      it!=(descriptor[i]->featNode.triangles).end(); )
        {
           if(it->average_cl<threshold || (it->average_anisotropy < 0.17 || it->average_anisotropy > 0.35) )
              it = (descriptor[i]->featNode.triangles).erase(it);
          else
              ++it;
        }
        numTriangles += descriptor[i]->featNode.triangles.size();
    }
}
