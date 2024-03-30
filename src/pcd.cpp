#include"PCL.h"

int viewPcd(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new 	pcl::PointCloud<pcl::PointXYZ>);
    if (-1 == pcl::io::loadPCDFile(path, *cloud)) //打开点云文件
    {
        std::cout << "error input!" << std::endl;
        return -1;
    }
    std::cout << cloud->points.size() << std::endl;
    // 初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Cloud Test"));
    //设置背景颜色为白色  0 0 0 
    viewer->setBackgroundColor(100, 100, 100);  
    // 对点云着色可视化 (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    // 等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}


void savePcd(std::string path) {
    pcl::PointCloud<pcl::PointXYZ> cloud;   // Fill in the cloud data  
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (std::size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII(path, cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test.pcd." << std::endl;
    for (std::size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
}

void pcdTest()
{
    int i = 0;
    std::cout << "--------------IO测试-------------" << std::endl;
    std::cout << "0.pcd保存测试" << std::endl;
    std::cout << "1.pcd读取测试" << std::endl;
    std::cout << "--------------IO测试-------------" << std::endl;
    std::cin >> i;
    switch (i)
    {
    case 0:
        savePcd("../data/test.pcd");
        break;
    case 1:
        viewPcd("../data/rabbit.pcd");
        break;
    default:
        std::cout << "编号输入错误 重新输入！" << std::endl;
        break;
    }
}
