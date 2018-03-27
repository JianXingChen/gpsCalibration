/**
* Copyright (C) 2017-2018 Zhaorui Zhang (iMorpheusAI)
* For more information see <https://github.com/iMorpheusAI/gpsCalibration>
*
* gpsCalibration is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#ifndef FEATUREMAPFUSION_H
#define FEATUREMAPFUSION_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/kdtree/kdtree.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "CellInfo.h"

//#define HEADANGLEXY (0)
//#define MAXITERTIMES (100) //修正Roll时最大迭代次数
//#define ITERSTEP (0.01) //修正Roll时迭代步长

using namespace std;
using namespace cv;


class FeatureMapFusion
{
public:
    FeatureMapFusion();
    ~FeatureMapFusion();

    //Feature map fusion
    static bool FusionCellsWithCells(const CellsENU& CellsENUSrc1, const CellsENU& CellsENUSrc2,
                                     CellsENU& targetCellsENU);

    //创建Kd-tree用来查找点对应的cell
    static pcl::KdTreeFLANN<pcl::PointXYZ> CreateKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCells);
};


#endif //FEATUREMAPFUSION_H
