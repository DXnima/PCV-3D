#include "boundary.h"

int estimateBorders(std::string path, double re, double reforn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    std::cout << "before size: " << cloud->size() << std::endl;

    pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);

    //定义一个法线估计的对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
    normEst.setRadiusSearch(reforn); //设置法线估计的半径//normEst.setKSearch(10);//表示计算点云法向量时，搜索的点云个数
    normEst.compute(*normals);       //将法线估计结果保存至normals
    //输出法线的个数
    std::cout << "reforn: " << reforn << std::endl;
    std::cerr << "normals: " << normals->size() << std::endl;

    //定义一个进行边界特征估计的对象
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
    boundEst.setInputCloud(cloud);                                                                             //设置输入的点云
    boundEst.setInputNormals(normals);                                                                         //设置边界估计的法线，因为边界估计依赖于法线
    boundEst.setRadiusSearch(re);                                                                              //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
    boundEst.setAngleThreshold(M_PI / 4);                                                                      //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
    boundEst.compute(boundaries);                                                                              //将边界估计结果保存在boundaries

    std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
    //输出边界点的个数
    std::cerr << "boundaries: " << boundaries.points.size() << std::endl;
    //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (boundaries[i].boundary_point > 0)
        {
            cloud_boundary->push_back(cloud->points[i]);
        }
    }
    pcl::io::savePCDFile<pcl::PointXYZ>("boundary.pcd", *cloud_boundary);
    return 0;
}

void boundaryTest()
{
    int i = 0;
    std::cout << "--------------边界测试-------------" << std::endl;
    std::cout << "0.提取边界" << std::endl;
    std::cout << "--------------边界测试-------------" << std::endl;
    std::cin >> i;
    std::cout << "输入pcd文件路径:" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (i)
    {
    case 0: {
        double re, reforn;
        std::cout << "设置边界估计所需的半径、法线估计的半径：" << std::endl;
        //设置边界估计所需的半径、法线估计的半径
        std::cin >> re >> reforn;
        estimateBorders(path, re, reforn);
        break;
    }
    default:
        std::cout << "编号输入错误 重新输入！" << std::endl;
        break;
    }

}
