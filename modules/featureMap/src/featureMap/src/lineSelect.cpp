#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 
#include <iostream>
#include <opencv/cv.h>
#include <fstream>
#include <vector>
#include <queue>

#include "featureMap/IMGNSS_Pointclouds2.h"
#include "featureMap/ChipLineInformation.h"
#include "featureMap/IMControl.h"
#include "featureMap/SVDPoints.h"
#include "featureMap/IMFeatureVector.h"

const float MAXDISTANCE = 0.25;

int lineNum = 0, lineNo = 0;

typedef struct{
    int lineNum;
    bool isStraight;
    double x;
    double y;
    double z;
} pointType;

typedef struct{
    int lineNum;
    double x;
    double y;
    double z;
    //double w;
    //double t;
} lvType;

lvType lvpoint;
std::vector<lvType> lvpoints;

std::vector<pointType> T, Direction, line_centroid;

int lineIndex = 0;

ros::Publisher pubLaserCloud;
std::vector<featureMap::SVDPoints> pubvector;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lineSelect_node");
    ros::NodeHandle nh;
    ros::Rate rate(1.0);    // set update rate
    rate.sleep();
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                         ("/velodyne_points", 2);
    ros::Publisher pubControl = nh.advertise<featureMap::IMControl>("/control_command",2);
    ros::Publisher pubSvd = nh.advertise<featureMap::SVDPoints>("/svdpoints",2);
    ros::Publisher pubFeatureVector = nh.advertise<featureMap::IMFeatureVector>("/feature_vector",2);

    //1.input line segment
    rosbag::Bag readbag;
    std::string inputFilename= argv[1];
    readbag.open(inputFilename, rosbag::bagmode::Read);

    featureMap::IMGNSS_Pointclouds2::ConstPtr gnss_pointcloud2;
    featureMap::SVDPoints svdpoints;
    geometry_msgs::Point point;
    std::vector<std::string> topics;
    topics.push_back(std::string("gnss_velodyne_points"));
    rosbag::View view(readbag, rosbag::TopicQuery(topics));

    pointType linepoint, direction, centroid;
    featureMap::IMFeatureVector vectorPub;

    for(rosbag::View::iterator it =  view.begin(); it != view.end(); ++it)
    {
        gnss_pointcloud2 = it->instantiate<featureMap::IMGNSS_Pointclouds2>();
        if (gnss_pointcloud2)
        {
            point.x = gnss_pointcloud2->pose.position.x;
            point.y = gnss_pointcloud2->pose.position.y;
            point.z = gnss_pointcloud2->pose.position.z;
            if(!gnss_pointcloud2->LineInformation.isStraight)//curve
            {
                //if a new curve coming
                if (lineNo != gnss_pointcloud2->LineInformation.lineNo)
                {
                    lineNum++;
                    lineNo = gnss_pointcloud2->LineInformation.lineNo;
                    std::cout << "curve lineNum: " << lineNum << std::endl;
                    //publish loam init msg
                    featureMap::IMControl controlMsg;
                    controlMsg.systemInited = false;
                    pubControl.publish(controlMsg);
                    rate.sleep();

                    if (!svdpoints.points.empty())
                    {
                        pubvector.push_back(svdpoints);       
                        svdpoints.points.clear();
                        svdpoints.poseWeights.clear();
                    }
                }
                //save curve rtk tracks
                svdpoints.lineNum = lineNum;
                svdpoints.isStraight = gnss_pointcloud2->LineInformation.isStraight;
                svdpoints.points.push_back(point);
                svdpoints.poseWeights.push_back(gnss_pointcloud2->poseWeight);
            }
            else//straight line
            {
                //if a new straight line coming
                if (lineNo != gnss_pointcloud2->LineInformation.lineNo)
                {
                    lineNum++;
                    lineNo = gnss_pointcloud2->LineInformation.lineNo;
                    std::cout << "straight line lineNum: " << lineNum << std::endl;

                    //publish loam init msg
                    featureMap::IMControl controlMsg;
                    controlMsg.systemInited = false;
                    pubControl.publish(controlMsg);
                    rate.sleep();

                    linepoint.lineNum = lineNum;
                    linepoint.x = point.x;
                    linepoint.y = point.y;
                    linepoint.z = point.z;
                    centroid.lineNum = lineNum;
                    centroid.x = gnss_pointcloud2->LineInformation.CentorPoint.x;
                    centroid.y = gnss_pointcloud2->LineInformation.CentorPoint.y;
                    centroid.z = gnss_pointcloud2->LineInformation.CentorPoint.z;
                    direction.lineNum = lineNum;
                    direction.isStraight = gnss_pointcloud2->LineInformation.isStraight;
                    direction.x = gnss_pointcloud2->LineInformation.DirectionVector.x;
                    direction.y = gnss_pointcloud2->LineInformation.DirectionVector.y;
                    direction.z = gnss_pointcloud2->LineInformation.DirectionVector.z;
                    T.push_back(linepoint);
                    Direction.push_back(direction);
                    line_centroid.push_back(centroid);

                    if (!svdpoints.points.empty())
                    {
                        pubvector.push_back(svdpoints);    
                        svdpoints.points.clear();
                        svdpoints.poseWeights.clear();
                    }
    
                    if (2 == Direction.size())
                    {
                        //2.plane fit
                        //eigen vlaue and eigen vector
                        Eigen::Vector3d u;
                        Eigen::Vector3d l;
                        Eigen::Vector3d v;//normal vector

                        v[0] = Direction[0].y * Direction[1].z - Direction[1].y * Direction[0].z;
                        v[1] = Direction[1].x * Direction[0].z - Direction[0].x * Direction[1].z;
                        v[2] = Direction[0].x * Direction[1].y - Direction[1].x * Direction[0].y;
                        double length = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

                        //make sure normal vector point up
                        if (v[2] > 0)
                        {
                            v[0] = v[0] / length;
                            v[1] = v[1] / length;
                            v[2] = v[2] / length;
                        }
                        else
                        {
                            v[0] = -1 * v[0] / length;
                            v[1] = -1 * v[1] / length;
                            v[2] = -1 * v[2] / length;
                        }
                        std::cout << "The ground normal vector in global coodinate is: " << v[0] << " " << v[1] << " " << v[2] << std::endl;

                        //if lines on the common plane
                        double distance = v[0] * (T[0].x - T[1].x) + v[1] * (T[0].y - T[1].y) + v[2] * (T[0].z - T[1].z);
                        if (distance < 0)
                        {
                            distance = -1 * distance;
                        }
                        //std::cout << "Distance:" << distance << std::endl;
    
                        if (distance > MAXDISTANCE)
                        {
                            std::cout << "************The two lines are not on the same plane!!!************" << std::endl;
                            //clear
                            T.clear();
                            Direction.clear();
                            line_centroid.clear();
                            continue;
                        }

                        u[0] = Direction[0].x;
                        u[1] = Direction[0].y;
                        u[2] = Direction[0].z;

                        l = v.cross(u);
                        std::cout << "The first rtk odometry heading direction: " << l[0] << " " << l[1] << " " << l[2] << std::endl;

                        lvpoint.lineNum = Direction[0].lineNum;
                        lvpoint.x = line_centroid[0].x + l[0];
                        lvpoint.y = line_centroid[0].y + l[1];
                        lvpoint.z = line_centroid[0].z + l[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[0].x - l[0];
                        lvpoint.y = line_centroid[0].y - l[1];
                        lvpoint.z = line_centroid[0].z - l[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[0].x + v[0];
                        lvpoint.y = line_centroid[0].y + v[1];
                        lvpoint.z = line_centroid[0].z + v[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[0].x - v[0];
                        lvpoint.y = line_centroid[0].y - v[1];
                        lvpoint.z = line_centroid[0].z - v[2];
                        lvpoints.push_back(lvpoint);
            
                    
                        //line 2
                        u[0] = Direction[1].x;
                        u[1] = Direction[1].y;
                        u[2] = Direction[1].z;

                        l = v.cross(u);
                        std::cout << "The second rtk odometry heading direction: " << l[0] << " " << l[1] << " " << l[2] << std::endl;

                        lvpoint.lineNum = Direction[1].lineNum;
                        lvpoint.x = line_centroid[1].x + l[0];
                        lvpoint.y = line_centroid[1].y + l[1];
                        lvpoint.z = line_centroid[1].z + l[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[1].x - l[0];
                        lvpoint.y = line_centroid[1].y - l[1];
                        lvpoint.z = line_centroid[1].z - l[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[1].x + v[0];
                        lvpoint.y = line_centroid[1].y + v[1];
                        lvpoint.z = line_centroid[1].z + v[2];
                        lvpoints.push_back(lvpoint);
                        lvpoint.x = line_centroid[1].x - v[0];
                        lvpoint.y = line_centroid[1].y - v[1];
                        lvpoint.z = line_centroid[1].z - v[2];
                        lvpoints.push_back(lvpoint);

                        vectorPub.DirectionVector.x= v[0];
                        vectorPub.DirectionVector.y= v[1];
                        vectorPub.DirectionVector.z= v[2];
                        vectorPub.CentorPoint.x= (line_centroid[0].x+ line_centroid[1].x)/ 2.0;
                        vectorPub.CentorPoint.y= (line_centroid[0].y+ line_centroid[1].y)/ 2.0;
                        vectorPub.CentorPoint.z= (line_centroid[0].z+ line_centroid[1].z)/ 2.0;

                        //clear
                        T.clear();
                        Direction.clear();
                        line_centroid.clear();
                    }
                }
                //save curve rtk tracks
                svdpoints.lineNum = lineNum;
                svdpoints.isStraight = gnss_pointcloud2->LineInformation.isStraight;
                svdpoints.points.push_back(point);
                svdpoints.poseWeights.push_back(gnss_pointcloud2->poseWeight);
            }
        
            pubLaserCloud.publish(gnss_pointcloud2->surround_points);
            rate.sleep();
        }
    }  

    //publish feature vector
    pubFeatureVector.publish(vectorPub);

    if (!svdpoints.points.empty())
    {
        pubvector.push_back(svdpoints);    
        svdpoints.points.clear();
        svdpoints.poseWeights.clear();
    }


    int nSize = lvpoints.size();
    for(int i = 0; i < nSize; i++)
    {
        point.x = lvpoints[i].x;
        point.y = lvpoints[i].y;
        point.z = lvpoints[i].z;
        pubvector[lvpoints[i].lineNum-1].points.push_back(point);
        pubvector[lvpoints[i].lineNum-1].poseWeights.push_back(0);
    }

    nSize = pubvector.size();
    for(int i = 0; i < nSize; i++)
    {
        pubSvd.publish(pubvector[i]);
        rate.sleep();
    }

    //publish loam init msg
    featureMap::IMControl controlMsg;
    controlMsg.systemInited = true;
    pubControl.publish(controlMsg);
    rate.sleep();

    readbag.close();

    return 0;
}

