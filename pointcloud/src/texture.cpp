#include "PCL.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>

#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
using namespace pcl;
using namespace pcl::io;

int textureMap(std::string path1, std::string path2)
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
    search::KdTree<PointXYZ>::Ptr tree;
    search::KdTree<PointNormal>::Ptr tree2;

    // add by ktran to test update functions
    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointNormal>::Ptr cloud_with_normals1(new PointCloud<PointNormal>);
    search::KdTree<PointXYZ>::Ptr tree3;
    search::KdTree<PointNormal>::Ptr tree4;

    // Load file
    pcl::PCLPointCloud2 cloud_blob;
    loadPCDFile(path1, cloud_blob);
    fromPCLPointCloud2(cloud_blob, *cloud);

    // Create search tree
    tree.reset(new search::KdTree<PointXYZ>(false));
    tree->setInputCloud(cloud);

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
    n.setInputCloud(cloud);
    //n.setIndices (indices[B);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate XYZ and normal information
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create search tree
    tree2.reset(new search::KdTree<PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Process for update cloud

    pcl::PCLPointCloud2 cloud_blob1;
    loadPCDFile(path2, cloud_blob1);
    fromPCLPointCloud2(cloud_blob1, *cloud1);
    // Create search tree
    tree3.reset(new search::KdTree<PointXYZ>(false));
    tree3->setInputCloud(cloud1);

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n1;
    PointCloud<Normal>::Ptr normals1(new PointCloud<Normal>());
    n1.setInputCloud(cloud1);

    n1.setSearchMethod(tree3);
    n1.setKSearch(20);
    n1.compute(*normals1);

    // Concatenate XYZ and normal information
    pcl::concatenateFields(*cloud1, *normals1, *cloud_with_normals1);
    // Create search tree
    tree4.reset(new search::KdTree<PointNormal>);
    tree4->setInputCloud(cloud_with_normals1);


    // Init objects
    PolygonMesh triangles;
    GreedyProjectionTriangulation<PointNormal> gp3;

    // Set parameters
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Reconstruct
    gp3.reconstruct(triangles);
    saveVTKFile("../data/mesh/bun0-gp3.vtk", triangles);
    // EXPECT_EQ(triangles.cloud.width, cloud_with_normals->width);
    // EXPECT_EQ(triangles.cloud.height, cloud_with_normals->height);
    // EXPECT_NEAR(int(triangles.polygons.size()), 685, 5);

    // // Check triangles
    // EXPECT_EQ(int(triangles.polygons.at(0).vertices.size()), 3);
    // EXPECT_EQ(int(triangles.polygons.at(0).vertices.at(0)), 0);
    // EXPECT_EQ(int(triangles.polygons.at(0).vertices.at(1)), 12);
    // EXPECT_EQ(int(triangles.polygons.at(0).vertices.at(2)), 198);
    // EXPECT_EQ(int(triangles.polygons.at(684).vertices.size()), 3);
    // EXPECT_EQ(int(triangles.polygons.at(684).vertices.at(0)), 393);
    // EXPECT_EQ(int(triangles.polygons.at(684).vertices.at(1)), 394);
    // EXPECT_EQ(int(triangles.polygons.at(684).vertices.at(2)), 395);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    int nr_points = cloud_with_normals->width * cloud_with_normals->height;
    // EXPECT_EQ(int(parts.size()), nr_points);
    // EXPECT_EQ(int(states.size()), nr_points);
    // EXPECT_EQ(parts[0], 0);
    // EXPECT_EQ(states[0], gp3.COMPLETED);
    // EXPECT_EQ(parts[393], 5);
    // EXPECT_EQ(states[393], gp3.BOUNDARY);

    // check if exist update cloud
    if (cloud_with_normals1->width * cloud_with_normals1->height > 0)
    {
        // Init objects
        PolygonMesh triangles;
        PolygonMesh triangles1;
        GreedyProjectionTriangulation<PointNormal> gp3;
        GreedyProjectionTriangulation<PointNormal> gp31;

        // Set parameters
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.setSearchRadius(0.025);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp3.setNormalConsistency(false);

        // for mesh 2
        // Set parameters
        gp31.setInputCloud(cloud_with_normals1);
        gp31.setSearchMethod(tree4);
        gp31.setSearchRadius(0.025);
        gp31.setMu(2.5);
        gp31.setMaximumNearestNeighbors(100);
        gp31.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp31.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp31.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp31.setNormalConsistency(false);

        // Reconstruct
        gp3.reconstruct(triangles);
        saveVTKFile("../data/mesh/bun01.vtk", triangles);

        gp31.reconstruct(triangles1);
        saveVTKFile("../data/mesh/bun02.vtk", triangles1);
    }

    if (cloud_with_normals1->width * cloud_with_normals1->height > 0)
    {
        // Init objects
        PolygonMesh triangles;
        PolygonMesh triangles1;
        GreedyProjectionTriangulation<PointNormal> gp3;
        GreedyProjectionTriangulation<PointNormal> gp31;

        // Set parameters
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.setSearchRadius(0.025);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp3.setNormalConsistency(false);

        gp3.reconstruct(triangles);

        // EXPECT_EQ(triangles.cloud.width, cloud_with_normals->width);
        // EXPECT_EQ(triangles.cloud.height, cloud_with_normals->height);
        // EXPECT_EQ(int(triangles.polygons.size()), 685);

        // update with texture mapping
        // set 2 texture for 2 mesh
        std::vector<std::string> tex_files;
        tex_files.emplace_back("../data/tex4.jpg");

        // initialize texture mesh
        TextureMesh tex_mesh;
        tex_mesh.cloud = triangles.cloud;

        // add the 1st mesh
        tex_mesh.tex_polygons.push_back(triangles.polygons);

        // update mesh and texture mesh
        //gp3.updateMesh(cloud_with_normals1, triangles, tex_mesh);
        // set texture for added cloud
        tex_files.push_back("../data/tex8.jpg");
        // save updated mesh
        saveVTKFile("../data/mesh/update_bunny.vtk", triangles);

        TextureMapping<PointXYZ> tm;

        // set mesh scale control
        tm.setF(0.01);

        // set vector field
        tm.setVectorField(1, 0, 0);

        TexMaterial tex_material;

        // default texture materials parameters
        tex_material.tex_Ka.r = 0.2f;
        tex_material.tex_Ka.g = 0.2f;
        tex_material.tex_Ka.b = 0.2f;

        tex_material.tex_Kd.r = 0.8f;
        tex_material.tex_Kd.g = 0.8f;
        tex_material.tex_Kd.b = 0.8f;

        tex_material.tex_Ks.r = 1.0f;
        tex_material.tex_Ks.g = 1.0f;
        tex_material.tex_Ks.b = 1.0f;
        tex_material.tex_d = 1.0f;
        tex_material.tex_Ns = 0.0f;
        tex_material.tex_illum = 2;

        // set texture material parameters
        tm.setTextureMaterials(tex_material);

        // set texture files
        tm.setTextureFiles(tex_files);

        // mapping
        tm.mapTexture2Mesh(tex_mesh);

        saveOBJFile("../data/mesh/update_bunny.obj", tex_mesh);
    }
    return 0;
}

void textureTest()
{
    std::string path = "../data/bun0.pcd";
    textureMap(path, path);
}