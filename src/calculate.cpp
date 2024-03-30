#include "calculate.h"

//读取平面参数 
Eigen::MatrixXf GetEquation(string path) {
    //读取平面参数
    int a = 8, b = 4;//定义一个8*4的矩阵，用于存放数据
    ifstream infile;//定义读取文件流，相对于程序来说是in
    infile.open(path);//打开文件
    Eigen::MatrixXf m_matrix(a, b);
    Eigen::VectorXf hang(b);
    for (int j = 0; j < a; j++)//共a 行
    {
        for (int i = 0; i < b; i++)//共b 列 组成一行
        {
            infile >> hang(i);
        }
        m_matrix.row(j) = hang;
    }
    infile.close();//读取完成之后关闭文件
    return m_matrix;
}

//获取xyz轴最大最小点
int getMinMax3DXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::console::TicToc time;
    time.tic();
    std::cerr << "Cloud: " << cloud->points.size() << std::endl;
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud, min, max);

    cout << "minXYZ: = " << min.x << "," << min.y << "," << min.z << "\n" << endl;
    cout << "maxXYZ: = " << max.x << "," << max.y << "," << max.z << "\n" << endl;

    cout << "获取xyz轴最大最小点用时： " << time.toc() / 1000 << " 秒" << endl;
    return 0;
}

int getMinMax3DXYZ(std::string path) {
    //加载点云模型
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    getMinMax3DXYZ(cloud);
    return 0;
}

//获取某个轴上最大值
static bool compare_x(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.x < b.x<0);
}
static bool compare_y(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.y < b.y<0);
}
static bool compare_z(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return (a.z < b.z<0);
}
int getMinMaxPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::console::TicToc time;
    time.tic();
    std::cerr << "Cloud: " << cloud->points.size() << std::endl;
    auto point_min_x = minmax_element(cloud->points.begin(), cloud->points.end(), compare_x); //返回迭代器
    pcl::PointXYZ xmin, xmax; xmin = *point_min_x.first; xmax = *point_min_x.second;
    cout << "X最小值点的坐标为: " << xmin << "\nX最大值点的坐标为: " << xmax << endl;

    auto point_min_y = minmax_element(cloud->points.begin(), cloud->points.end(), compare_y); //返回迭代器
    pcl::PointXYZ ymin, ymax; ymin = *point_min_y.first; ymax = *point_min_y.second;
    cout << "Y最小值点的坐标为: " << ymin << "\nY最大值点的坐标为: " << ymax << endl;

    auto point_min_z = minmax_element(cloud->points.begin(), cloud->points.end(), compare_z); //返回迭代器
    pcl::PointXYZ zmin, zmax; zmin = *point_min_z.first; zmax = *point_min_z.second;
    cout << "Z最小值点的坐标为: " << zmin << "\nZ最大值点的坐标为: " << zmax << endl;

    cout << "获取某个轴上最大值用时： " << time.toc() / 1000 << " 秒" << endl;
    // --------------------------结果可视化---------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setWindowName("计算点云的最大距离");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 241, 246, 242); // 蓝色
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");


    viewer->addArrow<pcl::PointXYZ>(xmin, xmax, 255, 0, 0, true, "arrow1", 0);
    viewer->addArrow<pcl::PointXYZ>(ymin, ymax, 0, 255, 0, true, "arrow2", 1);
    viewer->addArrow<pcl::PointXYZ>(zmin, zmax, 0, 0, 255, true, "arrow3", 2);
    viewer->addText3D("XPoint1", xmin, 0.05, 255, 0, 0);
    viewer->addText3D("XPoint2", xmax, 0.05, 255, 0, 0);

    viewer->addText3D("YPoint1", ymin, 0.05, 0, 255, 0);
    viewer->addText3D("YPoint2", ymax, 0.05, 0, 255, 0);

    viewer->addText3D("ZPoint1", zmin, 0.05, 0, 0, 255);
    viewer->addText3D("ZPoint2", zmax, 0.05, 0, 0, 255);

    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}

int getMinMaxPoint(string path)
{
    // ---------------------------加载点云数据--------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    getMinMaxPoint(cloud);
    return (0);
}

//计算前后平面方程
int PlaneIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::console::TicToc time;
    time.tic();
    Eigen::Vector4f plane_a, plane_b;
    Eigen::VectorXf line;
    Eigen::MatrixXf planes = GetEquation("../data/process/plane_equation.txt");
    plane_a = planes.row(0);
    plane_b = planes.row(2);
    //--------------------计算两平面的交线----------------------------
    pcl::planeWithPlaneIntersection(plane_a, plane_b, line, 1e-6);
    cout << "相交直线,过点(x0,y0,z0),方向向量是(m,n,p)：\n" << line << endl;
    auto point_min = minmax_element(cloud->points.begin(), cloud->points.end(), compare_x); //返回迭代器
    pcl::PointXYZ min, max; min = *point_min.first; max = *point_min.second;

    //d=-(mx0 + ny0 + pz0)
    float d1 = -(line(3) * min.x + line(4) * min.y + line(5) * min.z);
    float d2 = -(line(3) * max.x + line(4) * max.y + line(5) * max.z);
    //保存前后平面法向量
    ofstream out_txt_file;	// 定义/打开输出的txt文件
    out_txt_file.open("../data/process/plane_equation.txt", ios::app);
    out_txt_file << fixed;
    out_txt_file << line(3) << " " << line(4) << " " << line(5) << " " << d1 << "\n";
    out_txt_file << line(3) << " " << line(4) << " " << line(5) << " " << d2 << "\n";
    out_txt_file.close();	// 关闭文件
    cout << "计算前后平面用时： " << time.toc() / 1000 << " 秒" << endl;
    //-------------------------可视化---------------------------------
    //    // 初始化点云可视化对象
    //boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Cloud Test"));
    ////设置背景颜色为白色  0 0 0 
    //viewer->setBackgroundColor(100, 100, 100);
    //// 对点云着色可视化 (red).
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
    //viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    //// 可视化平面
    //pcl::ModelCoefficients plane, plane1;
    //for (size_t i = 3; i < 6; ++i)
    //{
    //    plane.values.push_back(line[i]);
    //    plane1.values.push_back(line[i]);
    //}
    //plane.values.push_back(d1);
    //plane1.values.push_back(d2);
    //viewer->addPlane(plane, "plane", 0);
    //viewer->addPlane(plane1, "plane1", 0);
    ////可视化最大最小点
    //viewer->addArrow<pcl::PointXYZ>(min, max, 255, 166, 0, true, "arrow", 0);
    //viewer->addText3D("MinPoint", min, 0.05, 17, 255, 0);
    //viewer->addText3D("MaxPoint", max, 0.05, 17, 255, 0);
    //viewer->addCoordinateSystem(0.1);
    //// 可视化平面交线
    /*pcl::PointXYZ p1, p2, p3, p4;
    p1.x = line[0]; p1.y = line[1]; p1.z = line[2];
    p2.x = line[3]; p2.y = line[4]; p2.z = line[5];
    viewer.addLine(p1, p2, 1, 0, 0, "line", 0);*/
    // 等待直到可视化窗口关闭
    //while (!viewer->wasStopped())
    //{
    //    viewer->spinOnce(100);
    //}
    return 0;
}

int PlaneIntersection(std::string path)
{
    //加载点云模型
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    PlaneIntersection(cloud);
    return 0;
}

//计算最大距离
int getMaxLength(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PointXYZ pmin, pmax;
    // 获取给定点云集合中的最大距离，并返回最小坐标点和最大坐标点。
    auto result = pcl::getMaxSegment(*cloud, pmin, pmax);
    cout << "点云集合中的最大距离为：" << result << "米" << endl;

    cout << "minpoint: = " << pmin.x << "," << pmin.y << "," << pmin.z << "\n" << endl;
    cout << "maxpoint: = " << pmax.x << "," << pmax.y << "," << pmax.z << "\n" << endl;

    cout << "计算最大距离用时： " << time.toc() / 1000 << " 秒" << endl;
    // --------------------------结果可视化---------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setWindowName("计算点云的最大距离");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 255); // 蓝色
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");

    viewer->addArrow<pcl::PointXYZ>(pmin, pmax, 0, 255, 0, true, "arrow", 0);
    viewer->addText3D("Point1", pmin, 0.05, 255, 0, 0);
    viewer->addText3D("Point2", pmax, 0.05, 255, 0, 0);


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}

int getMaxLength(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    getMaxLength(cloud);
    return 0;
}

//求三平面交点
Eigen::MatrixXf planeInter6(Eigen::MatrixXf coefficients)
{
    pcl::console::TicToc time;
    time.tic();

    // --------------------------计算交点---------------------------------
    // 0上 1下 2左 3右 4左斜 5右斜 6前 7后
    Eigen::Vector3f points[6];
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(4), coefficients.row(6), points[0], 1e-6);//上左斜
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(5), coefficients.row(6), points[1], 1e-6);//上右斜
    pcl::threePlanesIntersection(coefficients.row(3), coefficients.row(5), coefficients.row(6), points[2], 1e-6);//右斜
    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(3), coefficients.row(6), points[3], 1e-6);//下右
    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(2), coefficients.row(6), points[4], 1e-6);//下左
    pcl::threePlanesIntersection(coefficients.row(2), coefficients.row(4), coefficients.row(6), points[5], 1e-6);//左斜
    // --------------------------保存-------------------------------
    Eigen::MatrixXf mp(6, 3);
    for (size_t i = 0; i < 6; i++)
    {
        mp.row(i) = points[i];
    }
    cout << "求三平面交点用时： " << time.toc() / 1000 << " 秒" << endl;
    return mp;
}

Eigen::MatrixXf planeInter(Eigen::MatrixXf coefficients)
{
    pcl::console::TicToc time;
    time.tic();
    // --------------------------计算交点---------------------------------
    // 0上 1下 2左 3右 4左斜 5右斜 6前 7后
    Eigen::Vector3f points[12];
    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(2), coefficients.row(6), points[0], 1e-6);//下左
    pcl::threePlanesIntersection(coefficients.row(2), coefficients.row(4), coefficients.row(6), points[1], 1e-6);//左斜
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(4), coefficients.row(6), points[2], 1e-6);//上左斜
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(5), coefficients.row(6), points[3], 1e-6);//上右斜
    pcl::threePlanesIntersection(coefficients.row(3), coefficients.row(5), coefficients.row(6), points[4], 1e-6);//右斜
    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(3), coefficients.row(6), points[5], 1e-6);//下右

    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(2), coefficients.row(7), points[6], 1e-6);//下左
    pcl::threePlanesIntersection(coefficients.row(2), coefficients.row(4), coefficients.row(7), points[7], 1e-6);//左斜
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(4), coefficients.row(7), points[8], 1e-6);//上左斜
    pcl::threePlanesIntersection(coefficients.row(0), coefficients.row(5), coefficients.row(7), points[9], 1e-6);//上右斜
    pcl::threePlanesIntersection(coefficients.row(3), coefficients.row(5), coefficients.row(7), points[10], 1e-6);//右斜
    pcl::threePlanesIntersection(coefficients.row(1), coefficients.row(3), coefficients.row(7), points[11], 1e-6);//下右
    // --------------------------保存-------------------------------
    Eigen::MatrixXf mp(24, 3);
    int j = 0, k = 0;
    for (size_t i = 0; i < 24; )
    {
        if (j < 5) k = 0;
        else k = -6;
        mp.row(i) = points[j];//1782组成面
        mp.row(i + 1) = points[j + 6];
        mp.row(i + 2) = points[j + 7 + k];
        mp.row(i + 3) = points[j + 1 + k];
        i += 4;
        j += 1;
    }
    cout << "求三平面交点用时： " << time.toc() / 1000 << " 秒" << endl;
    return mp;
}

int planeInter()
{
    //读取平面参数
    Eigen::MatrixXf coefficients = GetEquation("../data/process/plane_equation.txt");
    Eigen::MatrixXf mp = planeInter(coefficients);

    std::ofstream fout;
    fout.open("../data/process/planeinter.txt", ios::out | ios::trunc);//在文件末尾追加写入
    fout << mp << std::endl;//每次写完一个矩阵以后换行
    fout.close();

    // --------------------------结果可视化-------------------------------
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //cloud.width = (uint32_t)12;
    //cloud.height = (uint32_t)1;
    //cloud.is_dense = false;
    //cloud.points.resize(cloud.width * cloud.height);
    //for (std::size_t i = 0; i < cloud.points.size(); ++i)
    //{
    //    cloud.points[i].x = points[i][0];
    //    cloud.points[i].y = points[i][1];
    //    cloud.points[i].z = points[i][2];
    //}
    //boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    //viewer->setBackgroundColor(0, 0, 0);
    //for (size_t i = 0; i < 12; ++i)
    //{
    //    cout << "交点为：\n" << points[i] << endl;
    //    pcl::PointXYZ o = { static_cast<float>(points[i][0]),static_cast<float>(points[i][1]),static_cast<float>(points[i][2]) };
    //    viewer->addSphere(o, 0.025, 1.0, 0.0, 0.0, "sphere");
    //    string str = "point" + i;
    //    viewer->addText3D(str, o, 0.05, 1.0, 0, 1.0);
    //}

    //while (!viewer->wasStopped())
    //{
    //    viewer->spinOnce(100);
    //    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //}

    return 0;
}

void calculateTest() {
    int i = 0;
    std::cout << "--------------几何计算-------------" << std::endl;
    std::cout << "0.计算前后平面方程" << std::endl;
    std::cout << "1.提取最大最小点" << std::endl;
    std::cout << "2.提取指定轴上最大最小点" << std::endl;
    std::cout << "3.计算最大距离" << std::endl;
    std::cout << "4.计算三面交点" << std::endl;
    std::cout << "--------------边界测试-------------" << std::endl;
    std::cin >> i;
    std::cout << "输入pcd文件路径:" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (i)
    {
    case 0: {
        path = "../data/process/cloud_uniform.pcd";
        PlaneIntersection(path);
        break;
    }
    case 1:
        getMinMax3DXYZ(path);
        break;
    case 2:
        getMinMaxPoint(path);
        break;
    case 3:
        getMaxLength(path);
        break;
    case 4:
        planeInter();
    default:
        std::cout << "编号输入错误 重新输入！" << std::endl;
        break;
    }
}
