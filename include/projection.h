#pragma once
#include "PCL.h"
#include <pcl/ModelCoefficients.h>       //模型系数定义头文件
#include <pcl/filters/project_inliers.h> //投影滤波类头文件
#include <pcl/common/transforms.h>       //  pcl::transformPointCloud 用到这个头文件
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>

using namespace std;
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//投影到XOY平面
int XYZPro(PointCloudT::Ptr cloud, char type);

//投影到XOY平面
int XOY(string path);

//投影到XOZ平面
int XOZ(string path);

//投影到YOZ平面
int YOZ(string path);

//投影到指定平面
PointCloudT::Ptr setPlane(PointCloudT::Ptr cloud, Eigen::Vector4f plane);
int setPlane(string path);

//计算到XYZ平面的角度
int planeAngle(Eigen::Vector3f plane);
//计算两平面的角度
int planeAngle();

//绕xyz轴旋转
pcl::PointCloud<pcl::PointXYZ>::Ptr transformationAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f plane);
int transformationAngle(string path, Eigen::Vector3f plane);

//变换矩阵变换点云
pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform);
int transformationMatrix(string path, Eigen::Matrix4f transform);

//求刚性变换矩阵
void rigid_transform_3D();
