#pragma once
#include "PCL.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

using namespace std;

//平面的分割
int PlaneModel(std::string path);

//实现圆柱体模型的分割：采用随机采样一致性估计从带有噪声的点云中提取一个圆柱体模型。
int CylinderMmodel(std::string path);

/******************************************************************************
 实现欧式聚类提取。对三维点云组成的场景进行分割
 打开点云数据，并对点云进行滤波重采样预处理，然后采用平面分割模型对点云进行分割处理
 提取出点云中所有在平面上的点集，并将其存盘
******************************************************************************/
int EuclideanCluster(std::string path);

// RANSAC拟合平面
pcl::PointCloud<pcl::PointXYZ> RandomSampleConsensus(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size);

//区域生长分割
int RegionGrowing(std::string path, int type);

// RANSAC拟合平面
int RandomSampleConsensus(std::string path);

int RANSACPlane(std::string path);