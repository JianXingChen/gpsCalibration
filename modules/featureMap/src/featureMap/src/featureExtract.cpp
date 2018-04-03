#include <opencv/cv.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <vector>
#include <queue>
#include <map>
#include <iostream>
#include <fstream>

#include "ENU2Local.h"
#include "CellInfo.h"
#include "FeatureMapFusion.h"
#include "featureMap/IMControl.h"
#include "featureMap/SVDPoints.h"
#include "featureMap/IMFeatureVector.h"

const float HEIGHT = 1.86;
const float RATIO = 500.0;
const float VERTICALITY = 0.008;
const float DEVIATION = 0.8;
const float ZERO = 0.000001;
const float CSIZE = 1.0;
const int MINSIZE = 200000;

int counter = 0;
int lineNum = 0;
bool systemInited = false;

typedef struct{
    double x;
    double y;
    double z;
    double w;
}rtkType;
rtkType rtkTrack;
std::vector<rtkType> vecRtkTrack;
std::map<bool, std::vector<rtkType> > mapRtkTrack;
std::map<int, std::map<bool, std::vector<rtkType> > > rtkTracks;

typedef struct{
    float x;
    float y;
    float z;
} loamType;
loamType loamTrack;
std::vector<loamType> vecLoamTracks;
std::map<int, std::vector<loamType> > loamTracks;//key:lineNum

pcl::PointXYZ point;
pcl::PointCloud<pcl::PointXYZ> loamMap;
std::map<int, pcl::PointCloud<pcl::PointXYZ> > loamMaps;//key:lineNum

char fileName[128] = { 0 };

Eigen::Matrix4d R;
Eigen::VectorXd p0(4);
Eigen::VectorXd p1(4);
cv::Mat matPoint(3, 1, CV_32F, cv::Scalar::all(0));

cv::Mat l(1, 3, CV_32F, cv::Scalar::all(0));
cv::Mat h(1, 3, CV_32F, cv::Scalar::all(0));
cv::Mat g(1, 3, CV_32F, cv::Scalar::all(0));
cv::Mat v3(1, 3, CV_32F, cv::Scalar::all(0));
cv::Mat temp(1, 3, CV_32F, cv::Scalar::all(0));

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudRotated(new pcl::PointCloud<pcl::PointXYZ>());

vector<pair<Eigen::Vector3d, pcl::PointXYZ>> vectorTotal;
CellsENU cloudCellTotal;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) 
{
    if ((fabs(laserOdometry->pose.pose.position.x) < ZERO) && (fabs(laserOdometry->pose.pose.position.y) < ZERO) && (fabs(laserOdometry->pose.pose.position.z) < ZERO))
    {
        lineNum++;
        std::cout << "LOAM tracks lineNum: " << lineNum << std::endl;

        vecLoamTracks.clear();
        loamTracks[lineNum] = vecLoamTracks;
    }
    loamTrack.x = laserOdometry->pose.pose.position.z;
    loamTrack.y = laserOdometry->pose.pose.position.x;
    loamTrack.z = laserOdometry->pose.pose.position.y;
    
    loamTracks[lineNum].push_back(loamTrack);
}

void controlHandler(const featureMap::IMControl::ConstPtr& msg) 
{
    if (msg->systemInited)
    {
        systemInited = true;
    }
}

void vectorHandler(const featureMap::IMFeatureVector::ConstPtr& featureVector) 
{
    pair<Eigen::Vector3d, pcl::PointXYZ> vectorTemp;
    vectorTemp.first(0)= featureVector->DirectionVector.x;
    vectorTemp.first(1)= featureVector->DirectionVector.y;
    vectorTemp.first(2)= featureVector->DirectionVector.z;
    vectorTemp.second.x= featureVector->CentorPoint.x;
    vectorTemp.second.y= featureVector->CentorPoint.y;
    vectorTemp.second.z= featureVector->CentorPoint.z;
    vectorTotal.push_back(vectorTemp);
    std::cout<< "test x is: "<< vectorTemp.second.x << std::endl;
}

void svdHandler(const featureMap::SVDPoints::ConstPtr& svdpoints) 
{
    int lineNum = svdpoints->lineNum;
    bool isStraight = svdpoints->isStraight;

    vecRtkTrack.clear();
    mapRtkTrack.clear();    

    size_t nSize = svdpoints->points.size();
    rtkType rtkPoint;
    for (size_t i = 0; i < nSize; i++)
    {
        rtkPoint.x = svdpoints->points[i].x;
        rtkPoint.y = svdpoints->points[i].y;
        rtkPoint.z = svdpoints->points[i].z;
        rtkPoint.w = svdpoints->poseWeights[i];
        vecRtkTrack.push_back(rtkPoint);
    }
    mapRtkTrack[isStraight] = vecRtkTrack;
    rtkTracks[lineNum] = mapRtkTrack;
}

void pointcloudHandler(const sensor_msgs::PointCloud2::ConstPtr& pointcloudMsg)
{
    laserCloudFullRes->clear();
    pcl::fromROSMsg(*pointcloudMsg, *laserCloudFullRes);
    
    size_t nSize = laserCloudFullRes->points.size();
    for(size_t i = 0; i < nSize; i++)
    {
        point.x = laserCloudFullRes->points[i].z;
        point.y = laserCloudFullRes->points[i].x;
        point.z = laserCloudFullRes->points[i].y;
        loamMaps[lineNum].points.push_back(point);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "featureExtract");
    ros::NodeHandle nh;
 
    ros::Subscriber odometrySub = nh.subscribe<nav_msgs::Odometry> ("/integrated_to_init", 5, odometryHandler);
    ros::Subscriber controlSub = nh.subscribe<featureMap::IMControl>("/control_command", 2, controlHandler);
    ros::Subscriber svdSub = nh.subscribe<featureMap::SVDPoints>("/svdpoints", 2, svdHandler);
    ros::Subscriber featureVector = nh.subscribe<featureMap::IMFeatureVector>("/feature_vector", 2, vectorHandler);
    ros::Subscriber pointcloudSub = nh.subscribe<sensor_msgs::PointCloud2>
                                        ("/velodyne_cloud_registered", 2, pointcloudHandler);

    std::string outputFilename= argv[1];
    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        if (systemInited)
        {
            systemInited = false;

            for(std::map<int, std::map<bool, std::vector<rtkType> > >::iterator it = rtkTracks.begin(); it != rtkTracks.end();)
            {
                std::cout << "The LOAM Odometry Linenum Is: " << it->first << std::endl;
                size_t size = it->second.begin()->second.size();
                std::map<int, std::vector<loamType> >::iterator loam_it = loamTracks.find(it->first);
                if (loam_it != loamTracks.end())//find the corresponding line num
                {
                    int nNum = it->first;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
    
                    std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator it_pointCloud = loamMaps.find(nNum);
                    if (it_pointCloud == loamMaps.end())
                    {
                        std::cout << "Can not find loam map!!!" << std::endl;
                        continue;
                    }
                    else
                    {
                        std::cout << "lineNum: " << nNum << ", and the pointcloud map size is: " << it_pointCloud->second.points.size() << std::endl;
                    }
                    pointCloudOrigin->points = it_pointCloud->second.points;

                    //TODO:
                    if (it->second.begin()->first)
                    {
                        size_t loamSize = loamTracks[nNum].size();
                        pcl::PointXYZ line_centroid;
                        line_centroid.x = 0;
                        line_centroid.y = 0;
                        line_centroid.z = 0;
                        cv::Mat matX(3, loamSize, CV_32F, cv::Scalar::all(0));
                        cv::Mat matW(loamSize, loamSize, CV_32F, cv::Scalar::all(0));
                        cv::Mat matXt(loamSize, 3, CV_32F, cv::Scalar::all(0));
                        cv::Mat matS(3, 3, CV_32F, cv::Scalar::all(0));

                        float totalWeight = 0.0;
                        for(size_t i =  0; i < loamSize - 1; i++)
                        {
                            matW.at<float>(i, i) = sqrt(pow((loamTracks[nNum][i+1].x - loamTracks[nNum][i].x), 2) + pow((loamTracks[nNum][i+1].y - loamTracks[nNum][i].y), 2) + pow((loamTracks[nNum][i+1].z - loamTracks[nNum][i].z), 2));
                            totalWeight += matW.at<float>(i, i);
                        }
                        matW.at<float>(loamSize - 1, loamSize - 1) = matW.at<float>(loamSize -2, loamSize - 2);
                        totalWeight += matW.at<float>(loamSize - 1, loamSize - 1);
                        //std::cout << "totalWeight: " << totalWeight << std::endl;

                        for(size_t i = 0; i < loamSize; i++)
                        {
                            matW.at<float>(i, i) = matW.at<float>(i, i) / totalWeight;
                            line_centroid.x += loamTracks[nNum][i].x * matW.at<float>(i, i);
                            line_centroid.y += loamTracks[nNum][i].y * matW.at<float>(i, i);
                            line_centroid.z += loamTracks[nNum][i].z * matW.at<float>(i, i);
                        }
    
                        //std::cout << "the centroid coordinate of local loam odometry is:" << std::endl;
                        //std::cout << line_centroid.x << " " << line_centroid.y << " " << line_centroid.z << std::endl;
                    
                        for(size_t i = 0; i < loamSize; i++)
                        {   
                            matX.at<float>(0, i) = loamTracks[nNum][i].x - line_centroid.x;
                            matX.at<float>(1, i) = loamTracks[nNum][i].y - line_centroid.y;
                            matX.at<float>(2, i) = loamTracks[nNum][i].z - line_centroid.z;
                        }   
                        cv::transpose(matX, matXt);
                        matS = matX * matW * matXt;

                        //eigen vlaue and eigen vector
                        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
                        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
                        cv::eigen(matS, matE, matV);

                        /*
                        std::cout << "Eigen value:" << matE.at<float>(0, 0) << " "  << matE.at<float>(0, 1) << " " << matE.at<float>(0, 2) << " " << std::endl;
                        std::cout << "Eigen Vector" << std::endl;
                        std::cout << matV.at<float>(0, 0) << " "  << matV.at<float>(0, 1) << " " << matV.at<float>(0, 2) << std::endl;
                        std::cout << matV.at<float>(1, 0) << " "  << matV.at<float>(1, 1) << " " << matV.at<float>(1, 2) << std::endl;
                        std::cout << matV.at<float>(2, 0) << " "  << matV.at<float>(2, 1) << " " << matV.at<float>(2, 2) << std::endl;
                        */

                        if (matE.at<float>(0, 0) / matE.at<float>(0, 1) < RATIO)
                        {   
                            std::cout << "******************######It's not a straight line!!!######*******************" << std::endl;
                        }
                            
                        if ((matV.at<float>(0, 0) * loamTracks[nNum][loamSize-1].x + matV.at<float>(0, 1) * loamTracks[nNum][loamSize-1].y + matV.at<float>(0, 2) * loamTracks[nNum][loamSize-1].z) > 0)
                        {   
                            h.at<float>(0, 0) = matV.at<float>(0, 0); 
                            h.at<float>(0, 1) = matV.at<float>(0, 1); 
                            h.at<float>(0, 2) = matV.at<float>(0, 2); 
                        }   
                        else
                        {   
                            h.at<float>(0, 0) = -1 * matV.at<float>(0, 0); 
                            h.at<float>(0, 1) = -1 * matV.at<float>(0, 1); 
                            h.at<float>(0, 2) = -1 * matV.at<float>(0, 2); 
                        }   

                        pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
                        pcl::PointCloud<pcl::PointXYZ> pointCloud;
                        pointCloudIn.points = it_pointCloud->second.points;

                        size_t size = pointCloudIn.points.size();
                        cv::Mat P(3, size, CV_32F, cv::Scalar::all(0));
                        cv::Mat Pt(size, 3, CV_32F, cv::Scalar::all(0));
#if 0
                        for(size_t i = 0; i < size; i++)
                        {
                            P.at<float>(0, i) = pointCloudIn->points[i].x;
                            P.at<float>(1, i) = pointCloudIn->points[i].y;
                            P.at<float>(2, i) = pointCloudIn->points[i].z;
                        }
                        cv::transpose(P, Pt);
                        matS = P * Pt;
                        cv::eigen(matS, matE, matV);

                        v3.at<float>(0, 0) = matV.at<float>(2, 0);
                        v3.at<float>(0, 1) = matV.at<float>(2, 1);
                        v3.at<float>(0, 2) = matV.at<float>(2, 2);
                        //normalize:l×(v3×l)
                        temp = v3.cross(h);
                        g = h.cross(temp);
#else//init g0
                        g.at<float>(0, 0) = 0;
                        g.at<float>(0, 1) = 0;
                        g.at<float>(0, 2) = 1;
#endif

                        float delta = 0.5;

                        counter = 0;
                        while(1)
                        {
                            counter++;
                            std::cout << "++++++++++++++++ Iterations To Find Ground Direction Of PCD +++++++++++++++" << std::endl;
                            pointCloud.clear();
                            for(size_t i = 0; i < size; i++)
                            {
                                if (fabs((line_centroid.x - pointCloudIn.points[i].x) * g.at<float>(0, 0) + (line_centroid.y - pointCloudIn.points[i].y) * g.at<float>(0, 1) + (line_centroid.z - pointCloudIn.points[i].z) * g.at<float>(0, 2) - HEIGHT ) < delta)
                                {
                                    pointCloud.points.push_back(pointCloudIn.points[i]);
                                }
                            }
                            size = pointCloud.points.size();
                            if(size < MINSIZE)
                            {
                                std::cout << "Point cloud size is too little!" << std::endl;
                                break;
                            }
                            std::cout << "Point cloud size:" << size << std::endl;
                            cv::Mat P(3, size, CV_32F, cv::Scalar::all(0));
                            cv::Mat Pt(size, 3, CV_32F, cv::Scalar::all(0));
    
                            for(size_t i = 0; i < size; i++)
                            {
                                P.at<float>(0, i) = pointCloudIn.points[i].x;
                                P.at<float>(1, i) = pointCloudIn.points[i].y;
                                P.at<float>(2, i) = pointCloudIn.points[i].z;
                            }
                            cv::transpose(P, Pt);
                            matS = P * Pt;
                            cv::eigen(matS, matE, matV);
                            v3.at<float>(0, 0) = matV.at<float>(2, 0); 
                            v3.at<float>(0, 1) = matV.at<float>(2, 1); 
                            v3.at<float>(0, 2) = matV.at<float>(2, 2);
                            //std::cout << "v3(n):" << v3.at<float>(0, 0) << " " << v3.at<float>(0, 1) << " " << v3.at<float>(0, 2) << std::endl;

                            //normalize:h×(v3×h)
                            temp = v3.cross(h);
                            g = h.cross(temp);
                            std::cout << "g(n):" << g.at<float>(0, 0) << " " << g.at<float>(0, 1) << " " << g.at<float>(0, 2) << std::endl;

                            float dot_product = fabs(v3.at<float>(0, 0) * h.at<float>(0, 0) + v3.at<float>(0, 1) * h.at<float>(0, 1) + v3.at<float>(0, 2) * h.at<float>(0, 2));
                            //std::cout << "dot product(v3 * h):" << dot_product << std::endl;
                            if (delta < 0.05 || dot_product < VERTICALITY)
                            {   
                                break;
                            }   
                            
                            /*
                            if (pointCloud->points.size() < size / 4)
                            {
                                std::cout << "Point cloud size reduces too many!" << std::endl;
                                break;
                            }
                            */

                            delta = 2.0 * delta / 3.0 ;

                            pointCloudIn.clear();
                            pointCloudIn.points = pointCloud.points;
                        }

                        if (g.at<float>(0, 2) < DEVIATION)//if the ground normal vector has a big deviation from the initial value
                        {
                            std::cout << "The result of g has a large deviation from the initial value: " << g.at<float>(0, 0) << " " << g.at<float>(0, 1) << " " << g.at<float>(0, 2) << std::endl;
                            g.at<float>(0, 0) = 0;
                            g.at<float>(0, 1) = 0;
                            g.at<float>(0, 2) = 1;
                        }

                        l = g.cross(h);

                        std::cout << "The local slam odometry heading direction is: " << h.at<float>(0, 0) << " "  << h.at<float>(0, 1) << " " << h.at<float>(0, 2) << std::endl;
                        std::cout << "The ground normal vector used is: " << g.at<float>(0, 0) << " " << g.at<float>(0, 1) << " " << g.at<float>(0, 2) << std::endl;
                    
                        cv::Mat matGlobal;
                        cv::Mat matLocal;
                        cv::Mat mat;
                        double weightSum = 0.0;

                        for(int i = 0; i < loamSize; i++)
                        {   
                            mat = (cv::Mat_<double>(1,3)<< loam_it->second[i].x, loam_it->second[i].y, loam_it->second[i].z);
                            matLocal.push_back(mat);
                        } 

                        mat = (cv::Mat_<double>(1,3)<< (double)(line_centroid.x + l.at<float>(0, 0)), (double)(line_centroid.y + l.at<float>(0, 1)), (double)(line_centroid.z + l.at<float>(0, 2)));
                        matLocal.push_back(mat);
                        mat = (cv::Mat_<double>(1,3)<< (double)(line_centroid.x - l.at<float>(0, 0)), (double)(line_centroid.y - l.at<float>(0, 1)), (double)(line_centroid.z - l.at<float>(0, 2)));
                        matLocal.push_back(mat);
                        mat = (cv::Mat_<double>(1,3)<< (double)(line_centroid.x + g.at<float>(0, 0)), (double)(line_centroid.y + g.at<float>(0, 1)), (double)(line_centroid.z + g.at<float>(0, 2)));
                        matLocal.push_back(mat);
                        mat = (cv::Mat_<double>(1,3)<< (double)(line_centroid.x - g.at<float>(0, 0)), (double)(line_centroid.y - g.at<float>(0, 1)), (double)(line_centroid.z - g.at<float>(0, 2)));
                        matLocal.push_back(mat);

                        int rtkSize = it->second.begin()->second.size();
                        //std::cout << "++++++++++++++++ rtkSize:" << rtkSize << std::endl;
                        for(int i = rtkSize - loamSize - 4; i < rtkSize; i++)
                        {
                            mat = (cv::Mat_<double>(1,3)<< it->second.begin()->second[i].x, it->second.begin()->second[i].y, it->second.begin()->second[i].z);
                            weightSum += it->second.begin()->second[i].w;
                            matGlobal.push_back(mat);
                        }

                        cv::Mat matWeight(loamSize + 4, loamSize + 4, CV_64F, cv::Scalar::all(0));
                        int i = 0;
                        for(; i < loamSize; i++)
                        {
                            matWeight.at<double>(i, i) = it->second.begin()->second[rtkSize - loamSize - 4 + i].w * 0.5 / weightSum;
                        }

                        for(int n = 0; n < 4; n++, i++)
                        {
                            matWeight.at<double>(i, i) = 0.25 * 0.5;
                        }

                        R = ENU2Local::ComputeTrMatrix(matLocal, matGlobal, matWeight);
                    }
                    else
                    {
                        cv::Mat matGlobal;
                        cv::Mat matLocal;
                        cv::Mat mat;
                        double weightSum = 0.0;
                        int loamSize = loam_it->second.size();
                        for(int i = 0; i < loamSize; i++)
                        {
                            mat = (cv::Mat_<double>(1,3)<< loam_it->second[i].x, loam_it->second[i].y, loam_it->second[i].z);
                            matLocal.push_back(mat);
                        }

                        int rtkSize = it->second.begin()->second.size();
                        for(int i = rtkSize - loamSize; i < rtkSize; i++)
                        {
                            mat = (cv::Mat_<double>(1,3)<< it->second.begin()->second[i].x, it->second.begin()->second[i].y, it->second.begin()->second[i].z);
                            weightSum += it->second.begin()->second[i].w;
                            matGlobal.push_back(mat);
                        }
                
                        cv::Mat matWeight(loamSize, loamSize, CV_64F, cv::Scalar::all(0));
                        for(int i = rtkSize - loamSize; i < rtkSize; i++)
                        {   
                            matWeight.at<double>(i + loamSize - rtkSize, i + loamSize - rtkSize) = it->second.begin()->second[i].w / weightSum;
                        }            
    
                        R = ENU2Local::ComputeTrMatrix(matLocal, matGlobal, matWeight); 
                    }
                    //std::cout << std::setprecision(12) << R << std::endl;
    
                    size_t pointCloudSize = pointCloudOrigin->points.size();
                    for(size_t i = 0; i < pointCloudSize; i++)
                    {
                        p0 << pointCloudOrigin->points[i].x, pointCloudOrigin->points[i].y, pointCloudOrigin->points[i].z, 1;
                        p1 = R * p0;
                        point.x = p1(0);
                        point.y = p1(1);
                        point.z = p1(2);
                        pointCloudRotated->points.push_back(point);
                    }
                    
                    //output feature map
                    if(nNum==1)
                    {
                        cloudCellTotal = CellsENU(pointCloudRotated, CSIZE);
                    }
                    else
                    {
                        CellsENU cloudCellInput = CellsENU(pointCloudRotated, CSIZE);
                        CellsENU cloudCellTarget;
                        FeatureMapFusion::FusionCellsWithCells(cloudCellInput, cloudCellTotal, cloudCellTarget);
                        cloudCellTotal= cloudCellTarget;
                    }
                    
                    //erase
                    rtkTracks.erase(it);
                    it = rtkTracks.begin(); 
                    continue;
                }
                ++it;
            }
            //TODO
            for(int iv= 0; iv< vectorTotal.size(); iv++ )
            {
                cloudCellTotal.AddVerticalVec_Point(vectorTotal[iv]);
            }
            cloudCellTotal.WriteCellsDataToFile(outputFilename.c_str());
        }

        status = ros::ok();
        rate.sleep();
    }

    return  0;
}
