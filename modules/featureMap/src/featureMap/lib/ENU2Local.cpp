#include <iostream>
#include <fstream>
#include "ENU2Local.h"

ENU2Local::ENU2Local()
{

}

ENU2Local::~ENU2Local()
{

}

//计算转换矩阵
Eigen::Matrix4d ENU2Local::ComputeTrMatrix(Mat localPoints, Mat globalPoints, Mat matW)
{
    Eigen::Matrix4d ret;
    if (localPoints.rows != globalPoints.rows || localPoints.rows != matW.rows)
    {
        std::cout << "Row size is not matching!" << std::endl;
        return ret;
    }

    //计算
    cv::Mat weightMatrixMat;
    Eigen::Matrix4d transformMatrix4d;
    weightMatrixMat.create(localPoints.rows, globalPoints.rows, CV_64FC1);

    for(int i = 0; i < localPoints.rows; i++)
    {
        for(int j = 0; j < globalPoints.rows; j++)
        {
            if(i==j)
                weightMatrixMat.at<double>(i,j) = matW.at<double>(i, j);
            else
                weightMatrixMat.at<double>(i,j) = 0.0;
        }
    }

    transformMatrix4d = ICPWithWeight3D(globalPoints, localPoints, weightMatrixMat);

    return transformMatrix4d;
}

//LOAM->ENU
Eigen::Matrix4d ENU2Local::ICPWithWeight3D(const cv::Mat &ENUPositions,
                                           const cv::Mat &LOAMPositions,
                                           const cv::Mat weightMat)
{
    Eigen::Matrix4d resultMatrix;
    resultMatrix << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    int posCnt = LOAMPositions.rows;
    if (posCnt != ENUPositions.rows)
    {
        cout<<"Error: wrong size of the inputs!\n";
        return resultMatrix;
    }

    cv::Mat ENUPositionsMat = ENUPositions.clone();
    cv::Mat LOAMPositionsMat = LOAMPositions.clone();

    //坐标系转换
    double _LOAMPositionsX = 0;
    double _LOAMPositionsY = 0;
    double _LOAMPositionsZ = 0;
    double _ENUPositionsX = 0;
    double _ENUPositionsY = 0;
    double _ENUPositionsZ = 0;
    double sigmaWeight = 0;
    for(int i = 0; i < posCnt; i++)
    {
        _LOAMPositionsX += LOAMPositions.at<double>(i,0) * weightMat.at<double>(i,i);
        _LOAMPositionsY += LOAMPositions.at<double>(i,1) * weightMat.at<double>(i,i);
        _LOAMPositionsZ += LOAMPositions.at<double>(i,2) * weightMat.at<double>(i,i);

        _ENUPositionsX += ENUPositions.at<double>(i,0) * weightMat.at<double>(i,i);
        _ENUPositionsY += ENUPositions.at<double>(i,1) * weightMat.at<double>(i,i);
        _ENUPositionsZ += ENUPositions.at<double>(i,2) * weightMat.at<double>(i,i);

        sigmaWeight += weightMat.at<double>(i,i);

    }
    _LOAMPositionsX /= sigmaWeight;
    _LOAMPositionsY /= sigmaWeight;
    _LOAMPositionsZ /= sigmaWeight;
    _ENUPositionsX /= sigmaWeight;
    _ENUPositionsY /= sigmaWeight;
    _ENUPositionsZ /= sigmaWeight;

    for(int i = 0; i < posCnt; i++)
    {
        LOAMPositionsMat.at<double>(i,0) -= _LOAMPositionsX;
        LOAMPositionsMat.at<double>(i,1) -= _LOAMPositionsY;
        LOAMPositionsMat.at<double>(i,2) -= _LOAMPositionsZ;

        ENUPositionsMat.at<double>(i,0) -= _ENUPositionsX;
        ENUPositionsMat.at<double>(i,1) -= _ENUPositionsY;
        ENUPositionsMat.at<double>(i,2) -= _ENUPositionsZ;
    }

    //Weighted ICP
    cv::Mat LOAMPositionsMatT = LOAMPositionsMat.t();//ENU2LOAM
    cv::Mat tempMat1 = weightMat * ENUPositionsMat;//LOAM2ENU
    cv::Mat tempMat2 = LOAMPositionsMatT * tempMat1;//ENU2LOAM
    cv::Mat w;
    cv::Mat u;
    cv::Mat vt;
    cv::SVD::compute(tempMat2, w, u, vt);
    //cout<<"svd::w= "<<w<<endl;
    //cout<<"svd::u= "<<u<<endl;
    //cout<<"svd::vt= "<<vt<<endl;
    if (!(w.at<double>(0,0) > 0 || w.at<double>(0,0) <= 0))
    {
        cout<<"Error: cv::SVD failed in ICPWithWeight3D!"<<endl;
        return resultMatrix;
    }

    cv::Mat A(3, 3, CV_64F, cv::Scalar::all(0));
    A.at<double>(0, 0) = 1;
    A.at<double>(1, 1) = 1;
    A.at<double>(2, 2) = cv::determinant(vt.t() * u.t());//u*vt->vt.t()*u.t()
    cv::Mat R = vt.t() * A * u.t();//u*A*vt->.
    cv::Mat _LOAMPositionsMat = (cv::Mat_<double> (3,1) <<
                                 _LOAMPositionsX,
                                 _LOAMPositionsY,
                                 _LOAMPositionsZ);
    cv::Mat _ENUPositionsMat = (cv::Mat_<double> (3,1) <<
                                 _ENUPositionsX,
                                 _ENUPositionsY,
                                 _ENUPositionsZ);
    cv::Mat t = _ENUPositionsMat - R * _LOAMPositionsMat;

    resultMatrix << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0),
                    0, 0, 0, 1;

    return resultMatrix;
}
