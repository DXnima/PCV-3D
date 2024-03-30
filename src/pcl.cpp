#include "PCL.h"
#include "calculate.h"
#include "process.h"
#include "bbox.h"
#include "projection.h"

void saveCloud(pcl::PointCloud<pcl::PointXYZ> cloud, string outpath) {
    if (!cloud.points.empty())
    {
        //保存
        pcl::io::savePCDFile<pcl::PointXYZ>(outpath, cloud, false);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void saveCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string outpath) {
    if (!cloud->points.empty())
    {
        //保存
        pcl::io::savePCDFile<pcl::PointXYZ>(outpath, *cloud, false);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string outpath) {
    if (cloud->width * cloud->height > 0)
    {
        //保存
        pcl::io::savePCDFile<pcl::PointXYZRGB>(outpath, *cloud);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void saveCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string outpath) {
    if (cloud->width * cloud->height > 0)
    {
        //保存
        pcl::io::savePCDFile<pcl::PointXYZI>(outpath, *cloud);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void saveCloud(pcl::PCLPointCloud2::Ptr cloud, string outpath) {
    if (cloud->width * cloud->height > 0)
    {
        //保存
        pcl::io::savePCDFile(outpath, *cloud);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void saveCloud(pcl::PointCloud<pcl::PointNormal> cloud, string outpath) {
    if (cloud.width * cloud.height > 0)
    {
        //保存
        pcl::io::savePCDFile(outpath, cloud);
        cout << "保存完成！" << endl;
    }
    else
        cout << "点云数据为空！" << endl;
}

void collectPCL() {

    int i = 0;
    cout << "--------------综合处理-------------" << endl;
    cout << "0.求前平面6个点" << endl;
    cout << "1.求前后平面12个点" << endl;
    cout << "--------------综合处理-------------" << endl;
    std::cin >> i;

    pcl::console::TicToc time;
    time.tic();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //声明变量
    pcl::io::loadPCDFile("../data/io/160m_clean.pcd", *cloud);//整体点云 
    //点云处理计算
    cloud = cloudProcess(cloud);
    //计算前后平面方程
    PlaneIntersection(cloud);
    //读取平面参数
    Eigen::MatrixXf planes = GetEquation("../data/process/plane_equation.txt");
    //计算前后平面距离
    float distance = abs(planes(6, 3) - planes(7, 3));
    //计算平面交点
    Eigen::MatrixXf mp;
    if (i == 0) mp = planeInter6(planes);
    else mp = planeInter(planes);
    //保存交点
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    new_cloud->width = mp.rows();
    new_cloud->height = 1;
    new_cloud->is_dense = false;
    new_cloud->points.resize(new_cloud->width * new_cloud->height);
    for (size_t i = 0; i < new_cloud->points.size(); ++i)
    {
        new_cloud->points[i].x = mp(i, 0);
        new_cloud->points[i].y = mp(i, 1);
        new_cloud->points[i].z = mp(i, 2);
    }
    //求变换矩阵
    Eigen::Matrix4f tm = getMatrix(cloud);
    //变换点云
    new_cloud = transformationMatrix(new_cloud, tm);
    //计算质心
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*new_cloud, center);
    //保存点
    int Num = new_cloud->points.size() + 2; //保存质心点和距离和六个拐角点
    float* X = new float[Num] {0.0};
    float* Y = new float[Num] {0.0};
    float* Z = new float[Num] {0.0};
    X[0] = distance; //第一行存距离
    X[1] = center(0); Y[1] = center(1); Z[1] = center(2); //第二行存中心点
    for (size_t i = 2; i < Num; ++i) ////第3行开始存点信息
    {
        X[i] = new_cloud->points[i-2].x;
        Y[i] = new_cloud->points[i-2].y;
        Z[i] = new_cloud->points[i-2].z;
    }
    ofstream fout;
    fout.open("../data/process/planeinter.txt", ios::out | ios::trunc);//写入文件
    for (int i = 0; i < Num; i++)
    {
        fout << X[i] << " " << Y[i] << " " << Z[i] << endl;
    }
    fout.close();
    //保存变换矩阵
    fout;
    fout.open("../data/process/matrix.txt", ios::out | ios::trunc);//写入文件
    fout << tm << endl;//每次写完一个矩阵以后换行
    fout.close();

    cout << "总用时： " << time.toc() / 1000 << " 秒" << endl;
}