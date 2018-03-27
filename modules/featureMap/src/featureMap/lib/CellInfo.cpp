/**
* Copyright (C) 2017-2018 Zhaorui Zhang (iMorpheusAI)
* For more information see <https://github.com/iMorpheusAI/gpsCalibration>
*
* gpsCalibration is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#include <algorithm>
#include <Eigen/Dense>
#include "CellInfo.h"

using namespace std;
using namespace cv;

CellsENU::CellsENU():
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>()),
    m_CellSize(CELLSIZE)
{

}

CellsENU::~CellsENU()
{

}

CellsENU::CellsENU(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud):
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>()),
    m_CellSize(CELLSIZE)
{
    CreateCells(inputCloud);
}

CellsENU::CellsENU(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, const float cellSize):
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>()),
    m_CellSize(cellSize)
{
    CreateCells(inputCloud);
}

CellsENU::CellsENU(const CellsENU& inputCellsENU):
    m_RepeatCellsNum(inputCellsENU.m_RepeatCellsNum),
    m_Cells(inputCellsENU.m_Cells),
    m_CellsKDTree(inputCellsENU.m_CellsKDTree),
    m_CellSize(inputCellsENU.m_CellSize),
    m_CoordRef(inputCellsENU.m_CoordRef),
    m_VerticalVec_Point(inputCellsENU.m_VerticalVec_Point),
    m_CellsENUServerData(inputCellsENU.m_CellsENUServerData)
{

}

float CellsENU::GetCellSize() const
{
    return m_CellSize;
}

void CellsENU::SetCellSize(float cellSize)
{
    m_CellSize = cellSize;
}

void CellsENU::AddCellData(pcl::PointXYZ cell, CellsENUServerData data)
{
    m_Cells->points.push_back(cell);
    m_CellsENUServerData.push_back(data);
}

void CellsENU::UpdateCellData(size_t num, CellsENUServerData data)
{
    if (num < m_CellsENUServerData.size())
    {
        m_CellsENUServerData[num].m_Mean = data.m_Mean;
        m_CellsENUServerData[num].m_N = data.m_N;
        m_CellsENUServerData[num].m_Confidence = data.m_Confidence;
        m_CellsENUServerData[num].m_Covariance = data.m_Covariance;
        m_CellsENUServerData[num].m_Cell.x = data.m_Cell.x;
        m_CellsENUServerData[num].m_Cell.y = data.m_Cell.y;
        m_CellsENUServerData[num].m_Cell.z = data.m_Cell.z;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CellsENU::GetCellsCloud() const
{
    return m_Cells;
}

vector<CellsENUServerData> CellsENU::GetCellsData() const
{
    return m_CellsENUServerData;
}

bool CellsENU::SortComp(const CellsNum &a, const CellsNum &b)
{
    if (a.m_X < b.m_X)
        return true;
    else if(a.m_X == b.m_X && a.m_Y < b.m_Y)
        return true;
    else if(a.m_X == b.m_X && a.m_Y == b.m_Y && a.m_Z < b.m_Z)
        return true;
    else
        return false;
}

void CellsENU::InsertCellsENUServerData(CellsENUServerData data, pcl::PointXYZ cell)
{
    data.m_Covariance = (data.m_Covariance -
                         ((data.m_Mean *
                           data.m_Mean.transpose()) /
                          data.m_N)) / (data.m_N - 1);
    data.m_Mean = data.m_Mean / data.m_N;
    data.m_Cell = cell;
    CellsENUServerData dataPush(data);
    m_CellsENUServerData.push_back(dataPush);
    m_Cells->points.push_back(cell);
}

void CellsENU::CreateCells(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    for(size_t i = 0; i < inputCloud->points.size(); i++)
    {
        pcl::PointXYZ p = inputCloud->points[i];

#ifdef CELLCVROUND
        CellsNum pInt(cvRound(p.x/m_CellSize)*m_CellSize,
                      cvRound(p.y/m_CellSize)*m_CellSize,
                      cvRound(p.z/m_CellSize)*m_CellSize,
                      i);
#else
        CellsNum pInt(float(int(p.x/m_CellSize)*m_CellSize),
                      float(int(p.y/m_CellSize)*m_CellSize),
                      float(int(p.z/m_CellSize)*m_CellSize),
                      i);
#endif
        m_RepeatCellsNum.push_back(pInt);
    }
    sort(m_RepeatCellsNum.begin(), m_RepeatCellsNum.end(), SortComp);//排序

    //构建Cells
    pcl::PointXYZ cellTemp(m_RepeatCellsNum[0].m_X,
                           m_RepeatCellsNum[0].m_Y,
                           m_RepeatCellsNum[0].m_Z);
    CellsENUServerData dataTemp;
    for(size_t i = 0; i < m_RepeatCellsNum.size(); i++)
    {
        if (m_RepeatCellsNum[i].m_X != cellTemp.x ||
            m_RepeatCellsNum[i].m_Y != cellTemp.y ||
            m_RepeatCellsNum[i].m_Z != cellTemp.z) //如果是新的cell
        {
            if (dataTemp.m_N > CELLPOINTCNTTH)
            {
                pcl::PointXYZ pCell(cellTemp.x, cellTemp.y, cellTemp.z);
                InsertCellsENUServerData(dataTemp, pCell);
            }
            dataTemp.ResetData();

            cellTemp.x = m_RepeatCellsNum[i].m_X;
            cellTemp.y = m_RepeatCellsNum[i].m_Y;
            cellTemp.z = m_RepeatCellsNum[i].m_Z;
        }

        {
            //pcl::PointXYZ pSrc = cloud_downSampled->points[m_RepeatCellsNum[i].m_Num];
            pcl::PointXYZ pSrc = inputCloud->points[m_RepeatCellsNum[i].m_Num];
            Eigen::Vector3d dataFm(pSrc.x, pSrc.y, pSrc.z);
            Eigen::Matrix3d dataSm = dataFm * dataFm.transpose();
            dataTemp.m_N++;
            dataTemp.m_NumInRepeatCells = i;
            dataTemp.m_Mean = dataTemp.m_Mean + dataFm;
            dataTemp.m_Covariance = dataTemp.m_Covariance + dataSm;
        }
    }
    if (dataTemp.m_N > CELLPOINTCNTTH)
    {
        cellTemp.x = m_RepeatCellsNum[m_RepeatCellsNum.size()-1].m_X;
        cellTemp.y = m_RepeatCellsNum[m_RepeatCellsNum.size()-1].m_Y;
        cellTemp.z = m_RepeatCellsNum[m_RepeatCellsNum.size()-1].m_Z;

        pcl::PointXYZ pCell(cellTemp.x, cellTemp.y, cellTemp.z);
        InsertCellsENUServerData(dataTemp, pCell);
    }
    dataTemp.ResetData();

    CreateKDTree();

    cout<<"There are "<<m_RepeatCellsNum.size()<<" points and "<<m_Cells->points.size()<<" cells created!"<<endl;

}

void CellsENU::CreateKDTree()
{
    m_CellsKDTree.setInputCloud(m_Cells);
}

pcl::KdTreeFLANN<pcl::PointXYZ> CellsENU::GetCellsKDTree() const
{
    return m_CellsKDTree;
}

void CellsENU::AddVerticalVec_Point(pair<Eigen::Vector3d, pcl::PointXYZ> verticalVec_Point)
{
    m_VerticalVec_Point.push_back(verticalVec_Point);
}

vector<pair<Eigen::Vector3d, pcl::PointXYZ> > CellsENU::GetVerticalVec_Points() const
{
    return m_VerticalVec_Point;
}

void CellsENU::ResetData()
{
    m_CellSize = CELLSIZE;
    m_RepeatCellsNum.clear();

    m_Cells->width = 1;
    m_Cells->height = 1;
    m_Cells->points.clear();
    m_CellsENUServerData.clear();
}

void CellsENU::operator=(const CellsENU& cellsData)
{
    this->m_CellSize = cellsData.m_CellSize;
    this->m_RepeatCellsNum = cellsData.m_RepeatCellsNum;
    this->m_Cells = cellsData.m_Cells;
    this->m_CellsENUServerData = cellsData.m_CellsENUServerData;
    this->m_CellsKDTree = cellsData.m_CellsKDTree;
    this->m_CoordRef = cellsData.m_CoordRef;
    this->m_VerticalVec_Point = cellsData.m_VerticalVec_Point;
}

bool CellsENU::ReadCellsDataFromFile(string filePath, long &coordRef_x, long &coordRef_y)
{
    cout << endl << "Reading cell feature data from " << filePath << " ..." << endl;

    ifstream ifile;
    ifile.open(filePath.c_str());
    if (!ifile.is_open())
    {
        cout<<"filePath in ReadCellFeatureData() can not open!"<<endl;
        return false;
    }

    size_t num = 0;
    bool dataStart = false;
    while(!ifile.eof())
    {
        string s;
        getline(ifile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            if (s.compare("#data") == 0)
                dataStart = true;
            else if (num == 1)//base
            {
                float cellSize = 0;
                ss >> cellSize;
                ss >> coordRef_x;
                ss >> coordRef_y;
                m_CellSize = cellSize;
                m_CoordRef.push_back(coordRef_x);
                m_CoordRef.push_back(coordRef_y);
            }
            else if((num >= 3) && (!dataStart))//vertical vector
            {
                double vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
                ss >> vx;
                ss >> vy;
                ss >> vz;
                ss >> px;
                ss >> py;
                ss >> pz;

                if (vx == 0 && vy == 0 && vz == 0)
                {
                    num++;
                    continue;
                }
                pair<Eigen::Vector3d, pcl::PointXYZ> verticalVec_Point;
                Eigen::Vector3d v(vx, vy, vz);
                pcl::PointXYZ p(px, py, pz);
                pair<Eigen::Vector3d, pcl::PointXYZ>(v, p);
                AddVerticalVec_Point(verticalVec_Point);
            }
            else if((num >= 5) && dataStart)//data
            {
                double cx, cy, cz, N, mean0, mean1, mean2;
                double cov00, cov01, cov02, cov11, cov12, cov22;

                ss >> cx;
                ss >> cy;
                ss >> cz;

                ss >> N;

                ss >> mean0;
                ss >> mean1;
                ss >> mean2;

                ss >> cov00;
                ss >> cov01;
                ss >> cov02;
                ss >> cov11;
                ss >> cov12;
                ss >> cov22;

                pcl::PointXYZ pCell(cx, cy, cz);
                Eigen::Vector3d mean(mean0, mean1, mean2);
                Eigen::Matrix3d cov;
                cov << cov00, cov01, cov02,
                       cov01, cov11, cov12,
                       cov02, cov12, cov22;
                CellsENUServerData data = CellsENUServerData(N, mean, cov, pCell);
                AddCellData(pCell, data);
            }

            num++;
        }
    }

    CreateKDTree();

    cout<<"Reading successfully completed."<<endl;
    return true;
}

//bool CellsENU::WriteCellsDataToFile(string filePath, long coordRef_x, long coordRef_y)
bool CellsENU::WriteCellsDataToFile(string filePath)
{
    cout <<"Saving cell feature data to " << filePath << " ..." << endl;

    ofstream of;
    of.open(filePath.c_str());
    if (!of.is_open())
    {
        cout<<"filePath in WriteCellFeatureData() can not open!"<<endl;
        return false;
    }

    //#base
    //of<<"#base"<<endl<<m_CellSize<<" "<<setprecision(12)<<coordRef_x<<" "<<coordRef_y<<endl;
    of<<"#base-cell-size"<<endl<<m_CellSize<<endl;

    //#vertical vector
    of<<"#ground-normal-vector"<<endl;
    if (m_VerticalVec_Point.size() > 0)
    {
        for(size_t k = 0; k < m_VerticalVec_Point.size(); k++)
        {
            of<<setprecision(9)
              <<m_VerticalVec_Point[k].first(0)<<" "<<m_VerticalVec_Point[k].first(1)<<" "<<m_VerticalVec_Point[k].first(2)<<" "
              <<m_VerticalVec_Point[k].second.x<<" "<<m_VerticalVec_Point[k].second.y<<" "<<m_VerticalVec_Point[k].second.z<<endl;
        }
    }
    else
        of<<"0 0 0 0 0 0"<<endl;
    //#data
    of<<"#cell-data"<<endl;
    std::vector<CellsENUServerData>& data = m_CellsENUServerData;
    if (data.size() > 0)
    {
        for(size_t i = 0; i < data.size(); i++)
        {
            of<<data[i].m_Cell.x<<" "<<data[i].m_Cell.y<<" "<<data[i].m_Cell.z<<" ";
            of<<setprecision(9)
              <<data[i].m_N<<" "<<data[i].m_Mean(0)<<" "<<data[i].m_Mean(1)<<" "<<data[i].m_Mean(2)<<" "
              <<data[i].m_Covariance(0,0)<<" "<<data[i].m_Covariance(0,1)<<" "<<data[i].m_Covariance(0,2)<<" "
              <<data[i].m_Covariance(1,1)<<" "<<data[i].m_Covariance(1,2)<<" "<<data[i].m_Covariance(2,2)<<endl;
        }
    }
    else
    {
        cout<<"CellsENU has no data to write!"<<endl;
        return false;
    }

    of.close();
    cout << "Cell feature data saved!" << endl;

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CellsENU::ResampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                            const vector<float> leafsize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (leafsize[0], leafsize[1], leafsize[2]);
    sor.filter (*cloud_filtered);

//    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." <<endl;
    return cloud_filtered;
}
