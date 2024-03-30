#pragma once
#include "PCL.h"
#include <fstream>
#include <cstdlib>
#include <map>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>          //贪婪投影三角化算法
#include <pcl/surface/poisson.h>      //泊松表面重建
#include <pcl/surface/convex_hull.h>  //凸包算法
#include <pcl/surface/concave_hull.h> //凹包算法
// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

using namespace std;

// PCL最小二乘（MLS）曲面平滑方法
int cloud_mls(std::string path);

// PCL贪婪投影三角化算法 无序点云的快速三角化
int mesh_gp3(std::string path);

//泊松表面重建
int mesh_poisson(std::string path);

//凸包重建
int mesh_convexhull(std::string path);

//凹包重建
int mesh_cavehull(std::string path);

//------------------CGAL重建-------------------------
// CGAL 点云平滑处理
typedef CGAL::Simple_cartesian<double> Kernel_WLOP;
typedef Kernel_WLOP::Point_3 Point;
typedef CGAL::Parallel_if_available_tag Concurrency_tag;

int cloud_wlop();

// CGAL 点云网格化
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_Mesh;
typedef Kernel_Mesh::FT FT;
typedef Kernel_Mesh::Point_3 Point_3;
typedef Kernel_Mesh::Vector_3 Vector_3;
typedef Kernel_Mesh::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

int cloud_mesh();
int cloud_mesh1();

// CGAL网格细化
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Vertex_handle Vertex_handle;
namespace PMP = CGAL::Polygon_mesh_processing;

int mesh_refinement();