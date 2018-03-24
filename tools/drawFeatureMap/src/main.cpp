#include "CellInfo.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>

#define COORDSTEP (20)

using namespace std;

CellsENU ReadCellInfo(const string filePath, const double coodRef_x, const double coodRef_y);
bool ReadCellFeatureData(string path, CellsENU &cellInfo);
double gaussionF(Eigen::Vector3d coord, Eigen::Vector3d mean, Eigen::Matrix3d cov);
void DrawCellsBMP(const vector<CellsENUServerData> &cellData, float cellSize, int imgWidth, int imgHeight,
                  int startCellX, int startCellY, double Z, int coordStep, string bmpSavePath);

int main(int argc, char *argv[])
{
    if(argc != 8)
    {
        cerr<<"Usage: ./DrawFeatureMap_tool path_to_feature_data IMG_save_path cell_start_X cell_start_Y IMG_cell_width IMG_cell_height Cell_Z"<<endl;
        return 1;
    }

    CellsENU read;
    double cellZ = std::stod(string(argv[7]));
    int coordStep = COORDSTEP;

    int imgWidth = std::stoi(string(argv[5])) * coordStep;
    int imgHeight = std::stoi(string(argv[6])) * coordStep;
    int cellStartX = std::stoi(string(argv[3]));
    int cellStartY = std::stoi(string(argv[4]));
    ReadCellFeatureData(string(argv[1]), read);
    vector<CellsENUServerData> data = read.GetCellsData();
    float cellSize = read.GetCellSize();

    DrawCellsBMP(data, cellSize, imgWidth, imgHeight, cellStartX, cellStartY, cellZ, coordStep,
                 string(argv[2]));

    return 0;
}

void DrawCellsBMP(const vector<CellsENUServerData> &cellData, float cellSize, int imgWidth,
                  int imgHeight, int startCellX, int startCellY, double Z, int coordStep,
                  string bmpSavePath)
{
    cv::Mat imgTmp = cv::Mat::zeros(imgHeight, imgWidth, CV_64FC1);
    cv::Mat img = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);
    double sMax = 0;
    for(size_t num = 0; num < cellData.size(); num++)
    {
        pcl::PointXYZ pCell = cellData[num].m_Cell;
        if (pCell.z == Z &&
            pCell.x > startCellX && pCell.x < (startCellX + imgWidth/coordStep) &&
            pCell.y > startCellY && pCell.y < (startCellY + imgHeight/coordStep))
        {
            int cellCol = cellData[num].m_Cell.x - startCellX;
            int cellRow = cellData[num].m_Cell.y - startCellY;
            for(int j = 0; j < coordStep; j++) //col
            {
                for(int k = 0; k < coordStep; k++) //row
                {
                    Eigen::Vector3d coord(-cellSize/2 + j * cellSize/coordStep + cellCol+startCellX,
                                          -cellSize/2 + k * cellSize/coordStep + cellRow+startCellY, Z);
                    double score = gaussionF(coord, cellData[num].m_Mean, cellData[num].m_Covariance);
                    if (score > sMax)
                        sMax = score;
                    imgTmp.at<double>(imgHeight - (coordStep*cellRow+k), coordStep*cellCol+j) = score;
                }
            }
        }
    }
    //cout<<"sMax = "<<sMax<<endl;
    if (sMax > 0)
    {
        for(int r = 0; r < imgTmp.rows; r++)
        {
            for(int c = 0; c < imgTmp.cols; c++)
            {
                img.at<uchar>(r, c) = uchar(imgTmp.at<double>(r, c) / sMax * 255);
            }
        }
        cv::imwrite(bmpSavePath, img);
    }
}

double gaussionF(Eigen::Vector3d coord, Eigen::Vector3d mean, Eigen::Matrix3d cov)
{
    double d3 = -log(0.55);
    double d1 = -log(5.05) - d3;
    double d2 = -2*log((0-log(4.5*exp(-0.5)+0.55)-d3)/d1);
    double tmp = -d1 * exp(-0.5 * d2 * (coord - mean).transpose() * cov.inverse() * (coord - mean));

    return tmp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ResampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                  const vector<float> leafsize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (leafsize[0], leafsize[1], leafsize[2]);
    sor.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." <<endl;

    return cloud_filtered;
}

CellsENU ReadCellInfo(const string filePath, const double coodRef_x, const double coodRef_y)
{
    CellsENU cellInfo = CellsENU();
    ifstream cellsFile(filePath);
    if (!cellsFile.is_open())
    {
        cout<<"filePath in ReadCellInfo() dose not exist!"<<endl;
        return cellInfo;
    }

    int lineNum = 0;
    while(!cellsFile.eof())
    {
        string s;
        getline(cellsFile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            if (lineNum == 0)
            {
                double size;
                ss >> size;
                cellInfo.SetCellSize(size);
            }
            else
            {
                double i, j, k, N, m1x, m1y, m1z, m200, m201, m202, m211, m212, m222;
                ss >> i;
                ss >> j;
                ss >> k;
                ss >> N;
                ss >> m1x;
                ss >> m1y;
                ss >> m1z;
                ss >> m200;
                ss >> m201;
                ss >> m202;
                ss >> m211;
                ss >> m212;
                ss >> m222;

                pcl::PointXYZ pCell(i - coodRef_x, j - coodRef_y, k);
                //cout<<"pCell = "<<pCell<<endl;
                Eigen::Vector3d m1(m1x - coodRef_x*N, m1y - coodRef_y*N, m1z);
                Eigen::Vector3d mean(m1x/N - coodRef_x, m1y/N - coodRef_y, m1z/N);
                Eigen::Matrix3d m2;
                double m200rec = m200 - coodRef_x*coodRef_x*N - 2*coodRef_x*m1(0);
                double m201rec = m201 - m1(0)*coodRef_x - m1(1)*coodRef_y - coodRef_x*coodRef_y*N;
                double m202rec = m202 - coodRef_x*m1(2);
                double m211rec = m211 - coodRef_y*coodRef_y*N - 2*coodRef_y*m1(1);
                double m212rec = m212 - coodRef_y*m1(2);
                m2 << m200rec, m201rec, m202rec,
                      m201rec, m211rec, m212rec,
                      m202rec, m212rec, m222;
                Eigen::Matrix3d cov = (m2 - (m1*m1.transpose()) / N) / (N-1);
                CellsENUServerData cellData = CellsENUServerData(N, mean, cov, pCell);
                cellInfo.AddCellData(pCell, cellData);

            }
        }
        lineNum++;
    }

    cellInfo.CreateKDTree();
    cellsFile.close();

    return cellInfo;
}

bool ReadCellFeatureData(string path, CellsENU &cellInfo)
{
    cout << endl << "Reading cell feature data from " << path << " ..." << endl;

    ifstream ifile;
    ifile.open(path.c_str());
    if (!ifile.is_open())
    {
        cout<<"filePath in ReadCellFeatureData() can not open!"<<endl;
        return false;
    }

    int num = 0;
    while(!ifile.eof())
    {
        string s;
        getline(ifile,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            if (num == 0)
            {
                float cellSize = 0;
                ss >> cellSize;
                cellInfo.SetCellSize(cellSize);
            }
            else
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
                cellInfo.AddCellData(pCell, data);
            }

            num++;
        }
    }

    cellInfo.CreateKDTree();

    return true;
}
