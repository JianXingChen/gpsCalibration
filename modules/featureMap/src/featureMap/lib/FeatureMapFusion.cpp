#include "FeatureMapFusion.h"


FeatureMapFusion::FeatureMapFusion()
{

}

FeatureMapFusion::~FeatureMapFusion()
{

}

bool FeatureMapFusion::FusionCellsWithCells(const CellsENU& CellsENUSrc1, const CellsENU& CellsENUSrc2,
                                            CellsENU& targetCellsENU)
{
    if ((CellsENUSrc1.GetCellSize() != CellsENUSrc2.GetCellSize()) || (CellsENUSrc1.GetCellSize() <= 0))
    {
        cout<<"Cell size in FusionCellsWithCells error!"<<endl;
        return false;
    }
    targetCellsENU.ResetData();
    targetCellsENU.SetCellSize(CellsENUSrc1.GetCellSize());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cell1 = CellsENUSrc1.GetCellsCloud();
    vector<CellsENUServerData> vf1data = CellsENUSrc1.GetCellsData();
    for(size_t i = 0; i < cell1->points.size(); i++)
    {
        CellsENUServerData d1 = CellsENUServerData(vf1data[i]);
        targetCellsENU.AddCellData(cell1->points[i], d1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cell2 = CellsENUSrc2.GetCellsCloud();
    vector<CellsENUServerData> vf2data = CellsENUSrc2.GetCellsData();
    for(size_t i = 0; i < cell2->points.size(); i++)
    {
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree = CellsENUSrc1.GetCellsKDTree();
        if (kdtree.nearestKSearch(cell2->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (pointNKNSquaredDistance[0] == 0)
            {
                CellsENUServerData d1 = CellsENUServerData(vf1data[pointIdxNKNSearch[0]]);
                CellsENUServerData d2 = CellsENUServerData(vf2data[i]);
                size_t N1 = d1.m_N;
                size_t N2 = d2.m_N;
                size_t N = N1 + N2;
                Eigen::Vector3d m11 = d1.m_Mean * N1;
                Eigen::Vector3d m12 = d2.m_Mean * N2;
                Eigen::Vector3d m1 = m11 + m12;
                Eigen::Vector3d mean = m1 / N;

                Eigen::Matrix3d m21 = d1.m_Covariance*(N1-1) + (m11*m11.transpose())/N1;
                Eigen::Matrix3d m22 = d2.m_Covariance*(N2-1) + (m12*m12.transpose())/N2;
                Eigen::Matrix3d m2 = m21 + m22;
                Eigen::Matrix3d cov = (m2 - ((m1*m1.transpose())/N)) / (N-1);

                pcl::PointXYZ pCell(cell2->points[i].x, cell2->points[i].y, cell2->points[i].z);
                CellsENUServerData dataUpdated = CellsENUServerData(N, mean, cov, pCell);
                targetCellsENU.UpdateCellData(pointIdxNKNSearch[0], dataUpdated);
            }
            else
            {
                CellsENUServerData d2New = CellsENUServerData(vf2data[i]);
                targetCellsENU.AddCellData(cell2->points[i], d2New);
            }
        }
        else
        {
            CellsENUServerData d2New = CellsENUServerData(vf2data[i]);
            targetCellsENU.AddCellData(cell2->points[i], d2New);
        }
    }

    targetCellsENU.CreateKDTree();

    cout<<"There are "<<cell1->points.size()<<" cells and "<<cell2->points.size()<<" cells fused, and "
        <<targetCellsENU.GetCellsCloud()->points.size()<<" cells created!"<<endl;

    return true;

}

//计算转换矩阵
Eigen::Matrix3d FeatureMapFusion::ComputeTrMatrix(const char* fileLocalCoord, const char* fileGlobalCoord)
{
    Eigen::Matrix3d ret;
    Mat localPoints;
    Mat globalPoints;

    //读取文件
    ifstream LocalFile(fileLocalCoord);
    if (!LocalFile.is_open())
    {
        cout<<"fileLocalCoord in ComputeTrMatrix() dose not exist!"<<endl;
        return ret;
    }
    ifstream GlobalFile(fileGlobalCoord);
    if (!GlobalFile.is_open())
    {
        cout<<"fileGlobalCoord in ComputeTrMatrix() dose not exist!"<<endl;
        return ret;
    }

    double x = 0, y = 0, z = 0;
    while(!LocalFile.eof())
    {
        string s;
        getline(LocalFile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            ss >> x;
            ss >> y;
            ss >> z;

            Mat pLocal = (cv::Mat_<double>(1,3)<<x, y, z);
            localPoints.push_back(pLocal);
        }
    }
    LocalFile.close();
    while(!GlobalFile.eof())
    {
        string s;
        getline(GlobalFile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            ss >> x;
            ss >> y;
            ss >> z;

            Mat pGlobal = (cv::Mat_<double>(1,3)<<x, y, z);
            globalPoints.push_back(pGlobal);
        }
    }
    GlobalFile.close();


    //计算
    cv::Mat weightMatrixMat = Mat::eye(localPoints.rows, localPoints.rows, CV_64FC1);
    Eigen::Matrix4d transformMatrix4d;
    for(int n = 0; n < localPoints.rows; n++)
        weightMatrixMat.at<double>(n,n) = 1.0;

    transformMatrix4d = ICPWithWeight3D(globalPoints, localPoints, weightMatrixMat);

    Eigen::Matrix3d transformMatrix3d;
    transformMatrix3d << transformMatrix4d(0,0), transformMatrix4d(0,1), transformMatrix4d(0,2),
                         transformMatrix4d(1,0), transformMatrix4d(1,1), transformMatrix4d(1,2),
                         transformMatrix4d(2,0), transformMatrix4d(2,1), transformMatrix4d(2,2);

    return transformMatrix3d;
}

//计算转换矩阵
Eigen::Matrix3d FeatureMapFusion::ComputeTrMatrix(const Eigen::Vector3d u, const Eigen::Vector3d v)
{
    Mat ENULineVec = (cv::Mat_<double>(1,3)<<u(0), u(1), u(2));
    Mat LidarUp = (cv::Mat_<double>(1,3)<<v(0), v(1), v(2));

    Mat axisZ = (cv::Mat_<double>(1,3)<<0, 0, -1);
    Mat axisX = (cv::Mat_<double>(1,3)<<1, 0, 0);
    //Mat LidarUp = ENULineVec.cross((axisZ.cross(ENULineVec)));

    Mat localCoord;
    localCoord.push_back(axisX);
    localCoord.push_back(axisZ);
    Mat ENUCoord;
    ENUCoord.push_back(ENULineVec);
    ENUCoord.push_back(LidarUp);

    cv::Mat weightMatrixMat = Mat::eye(localCoord.rows, localCoord.rows, CV_64FC1);
    Eigen::Matrix4d transformMatrix4d;
    for(int n = 0; n < localCoord.rows; n++)
        weightMatrixMat.at<double>(n,n) = 1.0;

    transformMatrix4d = ICPWithWeight3D(ENUCoord, localCoord, weightMatrixMat);

    Eigen::Matrix3d transformMatrix3d;
    transformMatrix3d << transformMatrix4d(0,0), transformMatrix4d(0,1), transformMatrix4d(0,2),
                         transformMatrix4d(1,0), transformMatrix4d(1,1), transformMatrix4d(1,2),
                         transformMatrix4d(2,0), transformMatrix4d(2,1), transformMatrix4d(2,2);

    return transformMatrix3d;
}

Eigen::Matrix3d FeatureMapFusion::ComputeTrMatrix(const Eigen::Vector3f u, const Eigen::Vector3f v)
{
    Eigen::Vector3d ud(u(0), u(1), u(2));
    Eigen::Vector3d vd(v(0), v(1), v(2));
    Eigen::Matrix3d ret = ComputeTrMatrix(ud, vd);
    return ret;
}

//LOAM->ENU
Eigen::Matrix4d FeatureMapFusion::ICPWithWeight3D(const cv::Mat& ENUPositions,
                                                  const cv::Mat& LOAMPositions,
                                                  const cv::Mat& weightMat)
{
    Eigen::Matrix4d resultMatrix;
    resultMatrix << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

    if (ENUPositions.type() != CV_64FC1 || LOAMPositions.type() != CV_64FC1 || weightMat.type() != CV_64FC1)
    {
        cout<<"Error: wrong type of the inputs in ICPWithWeight3D() !\n";
        return resultMatrix;
    }

    ///test
//    cout<<"ENUPositions = \n"<<ENUPositions<<endl;
//    cout<<"LOAMPositions = \n"<<LOAMPositions<<endl;
//    cout<<"weightMat = \n"<<weightMat<<endl;
    //end test

    int posCnt = LOAMPositions.rows;
    if (posCnt != ENUPositions.rows)
    {
        cout<<"Error: wrong size of the inputs in ICPWithWeight3D() !\n";
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

    //权重ICP
    cv::Mat ENUPositionsMatT = ENUPositionsMat.t();
    cv::Mat tempMat1 = weightMat * LOAMPositionsMat;
    cv::Mat tempMat2 = ENUPositionsMatT * tempMat1;
    cv::Mat w;
    cv::Mat u;
    cv::Mat vt;
    //cv::Mat S = (cv::Mat_<double> (3,3) << 2, 3, 5, 1 ,6, 5, 2, 7, 3, 9);
    //cout<<"ENUPositionsMatT = \n"<<ENUPositionsMatT<<endl;
    //cout<<"weightMat = \n"<<weightMat<<endl;
    //cout<<"LOAMPositionsMat = \n"<<LOAMPositionsMat<<endl;

    cv::SVD::compute(tempMat2, w, u, vt);
//    cout<<"tempMat2 = \n"<<tempMat2<<endl;
//    cout<<"svd::w = \n"<<w<<endl;
//    cout<<"u = \n"<<u<<endl;
//    cout<<"vt = \n"<<vt<<endl;
    if (!(w.at<double>(0,0) > 0 || w.at<double>(0,0) <= 0))
    {
        cout<<"Warning: cv::SVD failed in ICPWithWeight3D!"<<endl;
        return resultMatrix;
    }

    cv::Mat VUt = vt.t() * u.t();
    cv::Mat UVt = u * vt;
    double det = cv::determinant(UVt);
    cv::Mat detMat = (cv::Mat_<double> (3,3) <<1, 0, 0,
                                 0, 1, 0,
                                 0, 0, det);
    cv::Mat R = u * detMat * vt;
    //cv::Mat R = u * vt;
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

//SimpleLine FeatureMapFusion::SelectProperLine(const SimpleLine inputLine,
//                                              const vector<SimpleLine> lines)
//{

//}

pcl::KdTreeFLANN<pcl::PointXYZ> FeatureMapFusion::CreateKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCells)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inputCells);
    return kdtree;
}
