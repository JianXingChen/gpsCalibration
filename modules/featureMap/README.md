# iMorpheus.ai - high availability sub-meter precise GPS
## Feature Map
![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is needed by RTK position results and point cloud data.<br/>

## How to compile and run
### 1. Compile
```
$ cd featureMap
$ catkin_make
```
### 2. Run
1.*Make sure you have message bag. It includes specified message types, you can see more in featuremap/msg.
   If you don't have RTK and lidar, don't worry, we have sample data for your trial.*
##### Featue Map Demo Data -> [[download-92MB]](http://www.imorpheus.ai/download/dataForDemo/featureMapDemoData)

*Download the compressed data and put it into "modules/data/feartureMap" and type commands to decompress:*
```
$ tar -zvxf demo_data_for_feature_map.tar.gz
```

2.*Open the run.sh in directory "modules/featureMap" and set needed file directory correctly.*

3.*In directory "modules/featureMap", run commands:*

```
$ ./run.sh
```

4.*Finally, you get a feature map in global position system coordinates.*

5.*You can use the imaging tool we supplied in tools/drawFeatureMap to show the feature map.*

### 3. Example
  We show feature maps of demo data we have done. 

#### 3.1 Featur map results
![image](https://raw.githubusercontent.com/iMorpheusAI/gpsCalibration/develop/demo/demo.gif)
