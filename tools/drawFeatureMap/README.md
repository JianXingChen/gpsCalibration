# iMorpheus.ai - high availability sub-meter precise GPS
## Show Feature Map
![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is to show feature map built in module-featureMap.<br/>

## How to compile and run
### 1. Compile
```
$ cd drawFeatureMap
$ make
```
### 2. Run
1.*Make sure you have right feature data. It includes specified message types, you can see more in featuremap/msg.
   If you don't have feature data, don't worry, we have sample data for your trial.*
##### Featue Map Data -> [[download-92MB]](http://www.imorpheus.ai/download/dataForDemo/featureMapDemoData)

*Download the compressed data and put it into "tools/drawFeatureMap" and type commands to decompress:*
```
$ tar -zvxf demo_data_of_feature_map.tar.gz
```

2.*Open the run.sh in directory "modules/featureMap" and set needed file directory correctly.*

3.*In directory "modules/featureMap", run commands:*
```
$ ./run.sh
```

4.*Finally, you get an image of feature map in global position system coordinates.*

### 3. Example
  We show feature maps of demo data we have done. 

#### 3.1 Featur map results
![image](https://raw.githubusercontent.com/iMorpheusAI/gpsCalibration/develop/demo/demo.gif)
