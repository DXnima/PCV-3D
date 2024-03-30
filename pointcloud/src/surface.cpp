#include "surface.h"

// PCL最小二乘（MLS）曲面平滑方法
int cloud_mls(std::string path)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile(path, *cloud);
    std::cerr << "cloud size: " << cloud->points.size() << std::endl;
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.5);

    // Reconstruct
    mls.process(mls_points);
    std::cerr << "mls_points size: " << mls_points.points.size() << std::endl;
    if (mls_points.points.size() > 0)
    {
        pcl::PCDWriter writer;
        writer.write("../data/surface/mls.pcd", mls_points, false);
        std::cout << "保存完成！" << std::endl;
    }
    return 0;
}

// PCL贪婪投影三角化算法 无序点云的快速三角化
int mesh_gp3(std::string path)
{
    pcl::console::TicToc time;
    time.tic();
    // --------------------------------加载点云数据---------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(path, cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    //----------------------------------法线估计-----------------------------------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //-----------------------------连接XYZ和法向量字段-----------------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //-------------------------------定义搜索树对象--------------------------------
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    //-------------------------------贪婪投影三角化--------------------------------
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //定义三角化对象
    pcl::PolygonMesh triangles;                               // 存储最终三角化的网格模型
    gp3.setSearchRadius(0.015);                               // 设置连接点之间的最大距离（即三角形的最大边长）
    gp3.setMu(2.5);                                           // 设置被样本点搜索其临近点的最远距离，为了适应点云密度的变化
    gp3.setMaximumNearestNeighbors(100);                      // 设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(M_PI / 4);                     // 设置某点法线方向偏离样本点法线方向的最大角度
    gp3.setMinimumAngle(M_PI / 18);                           // 设置三角化后得到三角形内角的最小角度
    gp3.setMaximumAngle(2 * M_PI / 3);                        // 设置三角化后得到三角形内角的最大角度
    gp3.setNormalConsistency(false);                          // 设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud(cloud_with_normals); // 设置输入包含法线的点云
    gp3.setSearchMethod(tree2);            // 设置搜索方式
    gp3.reconstruct(triangles);            // 重建提取三角化

    cout << "贪婪投影三角化用时： " << time.toc() / 1000 << " 秒" << endl;
    // cout << triangles;
    pcl::io::savePLYFile("../data/surface/mesh_gp3.ply", triangles);
    
    // Finish
    return (0);
}

//泊松表面重建
int mesh_poisson(std::string path)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile(path, *cloud) == -1)
    {
        PCL_ERROR("Could not read pcd file!\n");
    }
    //----------------------------------法线估计------------------------------------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                         //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //-------------------------------连接法线和坐标---------------------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //---------------------------------泊松重建-------------------------------------
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    pn.setDepth(6); // 设置将用于表面重建的树的最大深度
    pn.setMinDepth(2);
    pn.setScale(1.25);           // 设置用于重建的立方体的直径与样本的边界立方体直径的比值
    pn.setSolverDivide(3);       // 设置块高斯-塞德尔求解器用于求解拉普拉斯方程的深度。
    pn.setIsoDivide(6);          // 设置块等表面提取器用于提取等表面的深度
    pn.setSamplesPerNode(10);    // 设置每个八叉树节点上最少采样点数目
    pn.setConfidence(false);     // 设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
    pn.setManifold(false);       // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); // 设置是否输出为多边形(而不是三角化行进立方体的结果)。
    cout << "泊松表面重建用时： " << time.toc() / 1000 << " 秒" << endl;
    //--------------------------------保存重建结果-----------------------------------
    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);
    pcl::io::savePLYFile("../data/surface/mesh_poisson.ply", mesh);
    return (0);
}

//凸包重建
int mesh_convexhull(std::string path)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(path, *cloud);

    //---------------------对上述点云构造凸包-----------------------
    pcl::ConvexHull<pcl::PointXYZ> hull; //创建凸包对象
    hull.setInputCloud(cloud);           //设置输入点云
    hull.setDimension(3);                //设置输入数据的维度(2D或3D)
    vector<pcl::Vertices> polygons;      //设置pcl:Vertices类型的向量，用于保存凸包顶点

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>); //该点云用于描述凸包形状

    hull.setComputeAreaVolume(true);           //设置为真，则调用qhr库来计算凸包的总面积和体积
    hull.reconstruct(*surface_hull, polygons); //计算3D凸包结果

    cout << "凸包重建用时： " << time.toc() / 1000 << " 秒" << endl;
    float Area = hull.getTotalArea();          //获取凸包的总面积
    float Volume = hull.getTotalVolume();      //获取凸包的总体积
    cout << "凸包的面积为：" << Area << endl;
    cout << "凸包的体积为：" << Volume << endl;
    pcl::PolygonMesh mesh;
    hull.reconstruct(mesh); // 重建面要素到mesh
    pcl::io::savePLYFile("../data/surface/mesh_convexhull.ply", mesh);
    return 0;
}

//凹包重建
int mesh_cavehull(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> cavehull;
    cavehull.setInputCloud(cloud);
    cavehull.setAlpha(0.003);
    vector<pcl::Vertices> polygons;
    cavehull.reconstruct(*surface_hull, polygons); // 重建面要素到点云

    pcl::PolygonMesh mesh;
    cavehull.reconstruct(mesh); // 重建面要素到mesh
    pcl::io::savePLYFile("../data/surface/mesh_cavehull.ply", mesh);
    cerr << "Concave hull has: " << surface_hull->points.size()
        << " data points." << endl;
    return (0);
}

// CGAL 点云平滑处理
int cloud_wlop()
{

    // 读取点云
    std::vector<Point> points;
    std::vector<Point> output;

    CGAL::IO::read_points("../data/process/cloud_sor.ply", std::back_inserter(points));

    std::cout << "size:" << points.size() << std::endl;
    //参数设置
    const double retain_percentage = 100; // 百分比
    const double neighbor_radius = 0.3;   //半径

    // WLOP 平滑处理
    CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>(points, std::back_inserter(output), CGAL::parameters::select_percentage(retain_percentage).neighbor_radius(neighbor_radius));

    //保存
    CGAL::IO::write_points("../data/process/rst.xyz", output, CGAL::parameters::stream_precision(5));

    return EXIT_SUCCESS;
}

// CGAL 点云网格化
int cloud_mesh()
{
    //加载点云
    Point_set points;
    std::ifstream stream("../data/process/rst.xyz", std::ios_base::binary);
    stream >> points;
    std::cout << "size: " << points.size() << " point(s)" << std::endl;

    //移除噪声
    typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(points, 30, points.parameters().threshold_percent(1)); //移除比例1%
    points.remove(rout_it, points.end());
    std::cout << points.number_of_removed_points() << " 个点被移除" << std::endl;
    points.collect_garbage();

    CGAL::IO::write_OFF("../data/process/sor.xyz", points);

    //点云平滑
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, 10);
    // CGAL::IO::write_OFF("smooth.off", points);

    typedef std::array<std::size_t, 3> Facet;
    std::vector<Facet> facets;

    CGAL::advancing_front_surface_reconstruction(points.points().begin(), points.points().end(), std::back_inserter(facets));
    std::cout << facets.size() << std::endl;

    std::vector<Point_3> vertices;
    vertices.reserve(points.size());
    std::copy(points.points().begin(), points.points().end(), std::back_inserter(vertices));
    CGAL::Surface_mesh<Point_3> output_mesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(vertices, facets, output_mesh);
    CGAL::IO::write_PLY("../data/surface/test.ply", output_mesh);

    return EXIT_SUCCESS;
}

int cloud_mesh1()
{
    //加载点云
    Point_set points;
    std::ifstream stream("../data/process/rst.xyz", std::ios_base::binary);
    stream >> points;
    std::cout << "size: " << points.size() << " point(s)" << std::endl;

    //移除噪声
    typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>(points, 30, points.parameters().threshold_percent(1)); //移除比例1%
    points.remove(rout_it, points.end());
    std::cout << points.number_of_removed_points() << " 个点被移除" << std::endl;
    points.collect_garbage();

    CGAL::IO::write_OFF("../data/process/sor.xyz", points);

    //点云平滑
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(points, 10);
    // CGAL::IO::write_OFF("smooth.off", points);

    typedef std::array<std::size_t, 3> Facet;
    std::vector<Facet> facets;

    CGAL::advancing_front_surface_reconstruction(points.points().begin(), points.points().end(), std::back_inserter(facets));
    std::cout << facets.size() << std::endl;

    std::vector<Point_3> vertices;
    vertices.reserve(points.size());
    std::copy(points.points().begin(), points.points().end(), std::back_inserter(vertices));
    CGAL::Surface_mesh<Point_3> output_mesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(vertices, facets, output_mesh);
    CGAL::IO::write_PLY("../data/surface/test.ply", output_mesh);

    return EXIT_SUCCESS;
}

// CGAL网格细化
int mesh_refinement()
{
    Polyhedron poly;
    PMP::IO::read_polygon_mesh("../data/surface/test.ply", poly);
    std::vector<Polyhedron::Facet_handle> new_facets;
    std::vector<Vertex_handle> new_vertices;

    PMP::refine(poly, faces(poly), std::back_inserter(new_facets), std::back_inserter(new_vertices), CGAL::parameters::density_control_factor(2.5));
    std::ofstream refined_off("../data/surface/refined.off");
    refined_off.precision(17);
    refined_off << poly;
    refined_off.close();
    std::cout << "重建完毕" << std::endl;
    return 0;
}

void surfaceTest()
{
    int i = 0;
    std::cout << "--------------表面重建测试-------------" << std::endl;
    std::cout << "0.最小二乘(MLS)曲面平滑方法" << std::endl;
    std::cout << "1.PCL贪婪三角化" << std::endl;
    std::cout << "2.PCL泊松重建" << std::endl;
    std::cout << "3.PCL凸包重建" << std::endl;
    std::cout << "4.PCL凹包重建" << std::endl;
    std::cout << "5.CGAL点云平滑" << std::endl;
    std::cout << "6.CGAL点云网格化" << std::endl;
    std::cout << "7.CGAL网格细化" << std::endl;
    std::cout << "8.CGAL重建" << std::endl;
    std::cout << "--------------表面重建测试-------------" << std::endl;
    std::cin >> i;
    std::cout << "输入pcd文件路径:" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (i)
    {
    case 0:
        cloud_mls(path);
        break;
    case 1:
        mesh_gp3(path);
        break;
    case 2:
        mesh_poisson(path);
        break;
    case 3:
        mesh_convexhull(path);
        break;
    case 4:
        mesh_cavehull(path);
        break;
    case 5:
        cloud_wlop();
        break;
    case 6:
        cloud_mesh();
        break;
    case 7:
        mesh_refinement();
        break;
    case 8:
    {
        std::cout << "----------------CGAL点云平滑-------------------" << std::endl;
        cloud_wlop();
        std::cout << "----------------CGAL点云三角化-------------------" << std::endl;
        cloud_mesh();
        std::cout << "----------------CGAL网格细化-------------------" << std::endl;
        mesh_refinement();
        break;
    }
    default:
        std::cout << "编号输入错误 重新输入！" << std::endl;
        break;
    }
}