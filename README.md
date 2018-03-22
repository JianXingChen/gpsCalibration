# iMorpheus.ai - high availability sub-meter precise GPS
Through algorithm fusion of multiple data sources from different sensors such as lidar, radar, camera, gps, imu and point cloud, iMorpheus.ai brings about an high availability precision GPS measurement to outdoor robotics developer. iMorpheus integrate a number of advanced algorithm such as slam, kalman filter, ICP, feature selection and Gaussian Process. 
#### We believe precise GPS signal should obtained by computing from cloud rather than measure the satellite, and soly software and cloud can solve the problem rather than expensive hardware. So that we committed into only software to solve the problem. 
![image](https://raw.githubusercontent.com/iMorpheusAI/gpsCalibration/develop/demo/demo.gif)
## Copyrights
![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is a software package operate under off-line mode and using point clould and GPS to give you accurate GPS. Each GPS signal produced also comes with confidence level. 

## Installation Environment

### 1. Operating System
Ubuntu 14.04, 16.04

### 2. Robot Operating System - ROS
ROS provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.
##### [Learn More](http://wiki.ros.org/ROS/Tutorials)

### 3. Point Cloud Library - PCL
PCL is a large scale, open source project for 2D/3D image and point cloud processing.
##### [Learn More](http://pointclouds.org/documentation/)

### 4. EIGEN
Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
##### [Learn More](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Quick Installation
### Download 
- Download by GIT
```
$ git clone https://github.com/iMorpheusAI/gpsCalibration.git
```
- Download by browser - [click to download-gpsCalibration.zip-2.1MB](https://github.com/iMorpheusAI/gpsCalibration/archive/master.zip)<br/>
  After download, you can type command to decompress:
  ```
  $ unzip gpsCalibration-master.zip -d gpsCalibration
  ```

### Intallation
Under 'gpsCalibration/install' directory, we provide install scripts for beginners and developers. The scripts include installing ROS, PCL and EIGEN. <br/>
a) Basic version (Recommended for general users.) - one hour installation time.<br/>
b) Professional version (Recommended for developers.) 

- First download this repertory from github to your local system. 
- For ubuntu 14.04 basic version install, type commands:
```
$ cd gpsCalibration
$ cd install
$ sudo ./install_u1404_basic.sh 
```

## How to compile and run the project
### 1. Compile
1.* In gpsCalibration module, you can:*
```
$ cd modules/gpsCalibration
$ catkin_make 
```
2.* In feature map building module, you can:*
```
$ cd modules/featureMap
$ catkin_make 
```
### 2. Run
1.*Make sure that you already have data in right format, you can download our demo data and havd a try.*<br/>
<br/>
2.*We have details of command operation and resluts we have done in each module.*<br/>

## Questions
  You can ask any question [here](https://github.com/iMorpheusAI/gpsCalibration/issues) or send us emails(product@imorpheus.ai).
