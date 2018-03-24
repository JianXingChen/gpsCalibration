#include "LineInfo.h"

SimpleLine::SimpleLine()
{
    ResetLine();
}

SimpleLine::SimpleLine(Eigen::Vector3d centerPoint, Eigen::Vector3d orientation, Eigen::Vector3d verticalOrt)
{
    m_CenterPoint(0) = centerPoint(0);
    m_CenterPoint(1) = centerPoint(1);
    m_CenterPoint(2) = centerPoint(2);
    m_Orientation(0) = orientation(0);
    m_Orientation(1) = orientation(1);
    m_Orientation(2) = orientation(2);
    m_VerticalOrt(0) = verticalOrt(0);
    m_VerticalOrt(1) = verticalOrt(1);
    m_VerticalOrt(2) = verticalOrt(2);
}

SimpleLine::~SimpleLine()
{

}

Eigen::Vector3d SimpleLine::GetLineCenterPoint() const
{
    return m_CenterPoint;
}

Eigen::Vector3d SimpleLine::GetLineOrientation() const
{
    return m_Orientation;
}

Eigen::Vector3d SimpleLine::GetVerticalOrientation() const
{
    return m_VerticalOrt;
}

void SimpleLine::SetLineCenterPoint(const Eigen::Vector3d centerPoint)
{
    m_CenterPoint = centerPoint;
}

void SimpleLine::SetLineOrientation(const Eigen::Vector3d orientation)
{
    m_Orientation = orientation;
}

void SimpleLine::SetVerticalOrientation(const Eigen::Vector3d verticalOrt)
{
    m_VerticalOrt = verticalOrt;
}

void SimpleLine::ResetLine()
{
    m_CenterPoint(0) = 0; m_CenterPoint(1) = 0; m_CenterPoint(2) = 0;
    m_Orientation(0) = 0; m_Orientation(1) = 0; m_Orientation(2) = 0;
    m_VerticalOrt(0) = 0; m_VerticalOrt(1) = 0; m_VerticalOrt(2) = 0;
}
