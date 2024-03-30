#pragma once
#include "PCL.h"
#include <math.h>
#include <algorithm>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>          
#include <pcl/console/time.h> 
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;

int estimateBorders(std::string path, double re, double reforn);