#pragma once
#include "PCL.h"
#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
typedef pcl::PointXYZ PointType;

Eigen::Matrix4f getMatrix(pcl::PointCloud<PointType>::Ptr cloud);

pcl::PointCloud<PointType>::Ptr transformMatrix(pcl::PointCloud<PointType>::Ptr cloud);

int MinBbox(pcl::PointCloud<PointType>::Ptr cloud);

int MinBbox();
