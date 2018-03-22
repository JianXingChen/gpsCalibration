# iMorpheus.ai - high availability sub-meter precise GPS

## gpsCalibration 

![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is a software package operate under off-line mode and using point clould and GPS to give you accurate GPS. Each GPS signal produced also comes with confidence level. 

## How to compile and run the project
### 1. Compile
```
$ cd gpsCalibration
$ catkin_make 
```
### 2. Run
1.*Make sure you have the message bag. it includes follow message types
  sensor_msgs/PointCloud2 and GPS coordinates recording your run trail, even 
  GPS is not accurate and not continuous in time.
  If you don’t have lidar or GPS data, don’t worry, we have sample data for your trial.*
##### small size demo data -> [[download-191MB]](http://www.imorpheus.ai/download/dataForDemo/smallSizeDemoData)
##### large size demo data -> [[download-2.6GB]](http://www.imorpheus.ai/download/dataForDemo/largeSizeDemoData)
*Download the compressed demo data, put it into gpsCalibration/data and type commands to decompress:*
```
$ tar -zvxf small_size_demo_data.tar.gz
$ tar -zvxf large_size_demo_data.tar.gz 
```
*After decompression of small_size_demo_data.tar.gz, you will see:*
```
$ miniDemo/
   ├── bag_0
   └── original_gps_data.txt
```
2.*Open the run.sh in directory "gpsCalibration/" and set needed file directory correctly.*
```
    User Parameters:
    Input filenames:
    1. bag_input_filename:
       input point cloud bag file list
       bag_input_filename= "./data/bag_list.txt"

    2. gps_input_filename:
       original GPS data with type GPRMC
       gps_input_filename= "./data/original_gps_data.txt"
    
    3. result_control:
       output KML files or publish messages to other nodes
       1- output KML files
       2- publish messages to other nodes   
       
    Output filenames:
    4. gps_original_filename:
       original GPS track type with KML format
       gps_original_filename=  "./data/original_gps_track.kml"
       
    5. gps_improved_filename:
       imporved accurate GPS track type with KML format
       gps_improved_filename=  "./data/calibrated_gps_track.kml"
```
3.*In directory gpsCalibration, run commands:*
```
$ source setup.sh
$ cd data
$ vi bag_list.txt
  modify the point cloud data bag path: 
  ./data/miniDemo/bag_0
$ cp ./miniDemo/original_gps_data.txt ./
```
4.*Okay, you can run command:*
```
$ ./run.sh
```
5.*Finally, you get a global position system coordinates matched with your run trail. It is accurate and reliable!*

### 3. Example
  We show calibrated results of large size demo data. 
#### 3.1 demo results
##### [Results Download-large_size_demo_result.tar.gz-133KB](http://www.imorpheus.ai/download/dataForDemo/largeSizeDemoResult)
Download compressed demo results and type commands to decompress: 
```
$ tar -zvxf large_size_demo_result.tar.gz
$ tree large_size_demo_result
  ├── calibrated_gps_track.kml
  └── original_gps_track.kml
```
Open these KML files in google earth, you can check your results.
##### [See More](http://www.imorpheus.ai/demo/)
## About system input and output
### 1. Input
#### 1.1 .bag
A bag is a file format in ROS for storing ROS message data.
 
#### 1.2 GPS
The GPSRMC is protocol of GPSRMC's communication:
$GPRMC,085223.136,A,3957.6286,N,11619.2078,E,0.06,36.81,180908,,,A\*57

### 2. Output
#### 2.1 KML Files
The results are stored in gpsCalibration/data. We provide calibrated GPS in KML formats.<br/>
You can download Google Earth [here](https://www.google.com/earth/download/ge/) and open KML files.
#### 2.2 Messages
The messages are shown in *gpsCalibration/src/gpsCalibration/msg*.<br/>
In your node.cpp, you should include head files first:
```
#include "gpsCalibration/IMMessage.h"
#include "gpsCalibration/IMGPS.h
```
and type codes in your node.cpp:
```
ros::Subscriber calibratedGPS= nh.subscribe<>("/imorpheus_gps", 1, GPSHandle);
void GPSHandle(const gpsCalibration::IMMessagePtr& GPSWithWeight)
{
	//Your process codes;
}
```
