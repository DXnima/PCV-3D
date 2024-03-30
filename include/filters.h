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

//ֱͨ�˲���
pcl::PointCloud<pcl::PointXYZ>::Ptr PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int PassThrough(std::string path);

// VoxelGrid�˲���
//ʹ�����ػ����񷽷�ʵ���²����������ٵ������ ���ٵ������ݣ���ͬʱ������Ƶ���״�������������׼�������ؽ�����״ʶ����㷨�ٶ��зǳ�ʵ��
pcl::PCLPointCloud2::Ptr VoxelGrid(pcl::PCLPointCloud2::Ptr cloud, double size);
int VoxelGrid(std::string path, double size);

//ʹ��statisticalOutlierRemoval�˲����Ƴ���Ⱥ��\������
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int number, double size);
int StatisticalOutlierRemoval(std::string path, int number, double size);

//�뾶�˲� ɾ�����������һ����Χ��û�дﵽ�㹻����ڵ��������ݵ�
pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r, int number);
int RadiusOutlierRemoval(std::string path, double r, int number);

//��ֵ�²���
pcl::PointCloud<pcl::PointXYZ>::Ptr UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r);
int UniformSampling(std::string path, double r);

//������
pcl::PointCloud<pcl::PointNormal> MovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double s_r, double u_r, double size);
int MovingLeastSquares(std::string path, double s_r, double u_r, double size);

//˫���˲� ����ʱ�ϳ���
pcl::PointCloud<pcl::PointXYZ>::Ptr BilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int BilateralFilter(std::string path);

//��������
pcl::PointCloud<pcl::PointXYZRGB>::Ptr conditionalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int* rgb);
int conditionalFilter(std::string path);

//����ɫ����
int rgbFilter();

int GaussianKernel(std::string path);

