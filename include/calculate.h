#pragma once
#include "PCL.h"
#include <pcl/common/intersections.h>

using namespace std;

//读取平面参数 
Eigen::MatrixXf GetEquation(string path);

//获取xyz轴最大最小点
int getMinMax3DXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int getMinMax3DXYZ(std::string path);
int getMinMaxPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int getMinMaxPoint(string path);

//计算前后平面方程
int PlaneIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int PlaneIntersection(std::string path);

//计算最大距离
int getMaxLength(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int getMaxLength(std::string path);

//求三平面交点
Eigen::MatrixXf planeInter6(Eigen::MatrixXf coefficients);
Eigen::MatrixXf planeInter(Eigen::MatrixXf coefficients);
int planeInter();