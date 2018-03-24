#ifndef LINEINFO_H
#define LINEINFO_H

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

//直线段简要信息
class SimpleLine
{
public:
    SimpleLine();
    SimpleLine(Eigen::Vector3d centerPoint, Eigen::Vector3d orientation, Eigen::Vector3d verticalOrt);
    ~SimpleLine();

    Eigen::Vector3d GetLineCenterPoint() const;
    Eigen::Vector3d GetLineOrientation() const;
    Eigen::Vector3d GetVerticalOrientation() const;

    void SetLineCenterPoint(const Eigen::Vector3d centerPoint);
    void SetLineOrientation(const Eigen::Vector3d orientation);
    void SetVerticalOrientation(const Eigen::Vector3d verticalOrt);
    void ResetLine();

private:
    Eigen::Vector3d m_CenterPoint;
    Eigen::Vector3d m_Orientation;
    Eigen::Vector3d m_VerticalOrt;
};

#endif //LINEINFO_H
