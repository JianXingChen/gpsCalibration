#ifndef ENU2LOCAL_H
#define ENU2LOCAL_H

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#define MATHPI (3.141592653)

using namespace std;
using namespace cv;

class ENU2Local
{
public:
    ENU2Local();
    ~ENU2Local();

    //计算转换矩阵
    static Eigen::Matrix4d ComputeTrMatrix(Mat localPoints, Mat globalPoints, Mat matW);

private:
    //手动ICP带权重
    static Eigen::Matrix4d ICPWithWeight3D(const cv::Mat &ENUPositions, const cv::Mat &LOAMPositions,
                                           const cv::Mat weightMat);
};


#endif //ENU2LOCAL_H
