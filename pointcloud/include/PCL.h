#pragma once
#include "main.h"

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h> 
#include <boost/thread/thread.hpp>

void saveCloud(pcl::PointCloud<pcl::PointXYZ> cloud, std::string outpath);
void saveCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outpath);
void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string outpath);
void saveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string outpath);
void saveCloud(pcl::PCLPointCloud2::Ptr cloud, std::string outpath);
void saveCloud(pcl::PointCloud<pcl::PointNormal> cloud, std::string outpath);

//PCL测试
void pcdTest();
//过滤测试
void filterTest();
//分割测试
void segmentationTest();
//表面重建测试
void surfaceTest();
//纹理测试
void textureTest();
//平面投影测试
void projectionTest();
//边界提取测试
void boundaryTest();
//点云处理
void processTest();
//几何计算相关
void calculateTest();
//包围盒
void bboxTest();
//综合处理
void collectPCL();