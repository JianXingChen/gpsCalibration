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

#include "LineInfo.h"
#include "CellInfo.h"

using namespace std;
using namespace cv;

class FeatureMapFusion
{
public:
    FeatureMapFusion();
    ~FeatureMapFusion();

    //Feature Map Fusion
    static bool FusionCellsWithCells(const CellsENU& CellsENUSrc1, const CellsENU& CellsENUSrc2,
                                     CellsENU& targetCellsENU);

    //计算转换矩阵
    static Eigen::Matrix3d ComputeTrMatrix(const char* fileLocalCoord, const char* fileGlobalCoord);
    static Eigen::Matrix3d ComputeTrMatrix(const Eigen::Vector3d u, const Eigen::Vector3d v);
    static Eigen::Matrix3d ComputeTrMatrix(const Eigen::Vector3f u, const Eigen::Vector3f v);

    //选择合适的直线段
    //static SimpleLine SelectProperLine(const SimpleLine inputLine,
    //const vector<SimpleLine> lines);

    //创建Kd-tree用来查找点对应的cell
    static pcl::KdTreeFLANN<pcl::PointXYZ> CreateKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCells);

private:
    //手动ICP带权重
    static Eigen::Matrix4d ICPWithWeight3D(const cv::Mat &ENUPositions, const cv::Mat &LOAMPositions,
                                           const cv::Mat &weightMat);
};

#endif //FEATUREMAPFUSION_H
