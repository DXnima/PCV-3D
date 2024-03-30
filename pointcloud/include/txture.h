#pragma once
#include "PCL.h"
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>

#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
using namespace pcl;
using namespace pcl::io;

int textureMap(std::string path1, std::string path2);