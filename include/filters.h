#pragma once
#include "PCL.h"
#include "calculate.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <cmath>
#define PI 3.14159265

//直通滤波器
pcl::PointCloud<pcl::PointXYZ>::Ptr PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int PassThrough(std::string path);

// VoxelGrid滤波器
//使用体素化网格方法实现下采样，即减少点的数量 减少点云数据，并同时保存点云的形状特征，在提高配准，曲面重建，形状识别等算法速度中非常实用
pcl::PCLPointCloud2::Ptr VoxelGrid(pcl::PCLPointCloud2::Ptr cloud, double size);
int VoxelGrid(std::string path, double size);

//使用statisticalOutlierRemoval滤波器移除离群点\噪声点
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int number, double size);
int StatisticalOutlierRemoval(std::string path, int number, double size);

//半径滤波 删除在输入点云一定范围内没有达到足够多近邻的所有数据点
pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r, int number);
int RadiusOutlierRemoval(std::string path, double r, int number);

//均值下采样
pcl::PointCloud<pcl::PointXYZ>::Ptr UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r);
int UniformSampling(std::string path, double r);

//增采样
pcl::PointCloud<pcl::PointNormal> MovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double s_r, double u_r, double size);
int MovingLeastSquares(std::string path, double s_r, double u_r, double size);

//双边滤波 （耗时较长）
pcl::PointCloud<pcl::PointXYZ>::Ptr BilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int BilateralFilter(std::string path);

//条件过滤
pcl::PointCloud<pcl::PointXYZRGB>::Ptr conditionalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int* rgb);
int conditionalFilter(std::string path);

//按颜色过滤
int rgbFilter();

int GaussianKernel(std::string path);

