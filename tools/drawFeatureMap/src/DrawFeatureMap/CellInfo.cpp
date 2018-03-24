#include <algorithm>
#include <Eigen/Dense>
#include "CellInfo.h"

using namespace std;
using namespace cv;

CellsENU::CellsENU():
    m_CellSize(CELLSIZE),
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>())
{

}

CellsENU::~CellsENU()
{

}

CellsENU::CellsENU(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud):
    m_CellSize(CELLSIZE),
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>())
{
    CreateCells(inputCloud);
}

CellsENU::CellsENU(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, const float cellSize):
    m_CellSize(cellSize),
    m_Cells(new pcl::PointCloud<pcl::PointXYZ>())
{
    CreateCells(inputCloud);
}

CellsENU::CellsENU(const CellsENU& inputCellsENU):
    m_CellSize(inputCellsENU.m_CellSize),
    m_Cells(inputCellsENU.m_Cells),
    m_CellsENUServerData(inputCellsENU.m_CellsENUServerData),
    m_CellsKDTree(inputCellsENU.m_CellsKDTree)
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

    //m_CellsKDTree = CreateKDTree(m_Cells);
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

void CellsENU::ResetData()
{
    m_CellSize = CELLSIZE;
    m_RepeatCellsNum.clear();

    m_Cells->width = 1;
    m_Cells->height = 1;
    m_Cells->points.clear();
    m_CellsENUServerData.clear();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CellsENU::ResampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                            const vector<float> leafsize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (leafsize[0], leafsize[1], leafsize[2]);
    sor.filter (*cloud_filtered);

    return cloud_filtered;
}
