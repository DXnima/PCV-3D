#pragma once
#include "PCL.h"                                     
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/concave_hull.h>   

using namespace std;

//ransac平面拟合
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf& coefficient, float distance);

//直通
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
	float min, float max, string axis, bool is_save);

//平面分割
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative);

//删除重复点
pcl::PointCloud<pcl::PointXYZ>::Ptr delete_repeat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//统计滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_sor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int num, double stdmt);

//均匀
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_uniform_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float leaf_size);

//投影滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
	float paraA, float paraB, float paraC, float paraD);

//投影滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::Vector4f& coefficient);

// 计算法线
pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//计算alpha shapes平面点云边界特征提取
pcl::PointCloud<pcl::PointXYZ>::Ptr shapes_boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//PCL点云处理
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void cloudProcess();

//平面切割
void cloudBoundaryClip(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f coefficients[], string out_file);

//计算过滤后的各面投影并提取边界
void cloudProBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void cloudProBoundary(string path);

//计算前后面
pcl::PointCloud<pcl::PointXYZ>::Ptr aroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void aroundPlane();