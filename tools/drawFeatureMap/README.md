# iMorpheus.ai - high availability sub-meter precise GPS
## Draw Feature Map
![License](https://img.shields.io/badge/License-Apache2.0-blue.svg)

## Current Version - alpha
The alpha version is to show feature map in 2d.<br/>

## How to compile and run
### 1. Compile
```
$ mkdir build
$ cmake ../
$ make
```
### 2. Run
1.*Make sure you have sucessed in building feature map.
   If you haven't built it, don't worry, we have sample data for your trial.*
##### Featue Map Result -> [[download-482KB]](http://www.imorpheus.ai/download/dataForDemo/featureMapResult)

*Download the compressed data and put it into "modules/data/feartureMap" and type commands to decompress:*
```
$ tar -zvxf feature_map_result.tar.gz
```

2.*Open the run.sh in directory "tools/drawFeatureMap" and set needed file directory correctly.*

3.*In directory "tools/drawFeatureMap", run commands:*
```
$ ./run.sh
```

4.*Finally, you can see the feature map in 2d image.*


### 3. Example
  We show a result we have done. 

#### 3.1 Featur map results
![image](https://raw.githubusercontent.com/taichenliu/gpsCalibration/develop/demo/img.bmp)
