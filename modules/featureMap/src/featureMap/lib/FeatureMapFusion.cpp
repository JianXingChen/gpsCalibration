/**
* Copyright (C) 2017-2018 Zhaorui Zhang (iMorpheusAI)
* For more information see <https://github.com/iMorpheusAI/gpsCalibration>
*
* gpsCalibration is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

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

pcl::KdTreeFLANN<pcl::PointXYZ> FeatureMapFusion::CreateKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCells)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inputCells);
    return kdtree;
}
