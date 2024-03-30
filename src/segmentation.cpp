#include "segmentation.h"

//平面的分割
int PlaneModel(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl; //打印
    //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    //设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.01);        //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                           //表示点到估计模型的距离最大值，

    seg.setInputCloud(cloud);
    //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    //打印出平面模型
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    return (0);
}

//实现圆柱体模型的分割：采用随机采样一致性估计从带有噪声的点云中提取一个圆柱体模型。
int CylinderMmodel(std::string path)
{
    typedef pcl::PointXYZ PointT;
    // All the objects needed
    pcl::PCDReader reader;                                    // PCD文件读取对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;        //创建滤波器对象
    pcl::NormalEstimation<PointT, pcl::Normal> ne;            //法线估计对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //分割对象
    pcl::PCDWriter writer;                                    // PCD文件读取对象
    pcl::ExtractIndices<PointT> extract;                      //点提取对象
    pcl::ExtractIndices<pcl::Normal> extract_normals;         ///点提取对象
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    sor.setInputCloud(cloud);    //设置待滤波的点云
    sor.setMeanK(50);            //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阀值
    sor.setNegative(true);       // true：滤波结果取反，被过滤的点
    sor.filter(*cloud_filtered); //存储
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    //获取平面模型的系数和处在平面的内点
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // 从点云中抽取分割的处在平面上的点集
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // 存储分割得到的平面上的点到点云文件
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
    writer.write("../data/segmentation/CylinderMmodel_plane.pcd", *cloud_plane, false); // 分割得到的平面
    std::cout << "保存完成！" << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);        //设置对估计模型优化
    seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱形
    seg.setMethodType(pcl::SAC_RANSAC);       //参数估计方法
    seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
    seg.setMaxIterations(10000);              //设置迭代的最大次数10000
    seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值
    seg.setRadiusLimits(0, 0.1);              //设置估计出的圆柱模型的半径的范围
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
        writer.write("../data/segmentation/CylinderMmodel_cylinder.pcd", *cloud_cylinder, false); // 分割得到的平面
        std::cout << "保存完成！" << std::endl;
    }
    return (0);
}

/******************************************************************************
 实现欧式聚类提取。对三维点云组成的场景进行分割
 打开点云数据，并对点云进行滤波重采样预处理，然后采用平面分割模型对点云进行分割处理
 提取出点云中所有在平面上的点集，并将其存盘
******************************************************************************/
int EuclideanCluster(std::string path)
{
    //--------------------------读取桌面场景点云---------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
    cout << "读取点云: " << cloud->points.size() << " 个." << endl;

    //---------------------------体素滤波下采样----------------------------------
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    cout << "体素滤波后还有: " << cloud_filtered->points.size() << " 个." << endl;

    //--------------------创建平面模型分割的对象并设置参数-----------------------
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 分割模型,平面模型
    seg.setMethodType(pcl::SAC_RANSAC);    // 参数估计方法,随机采样一致性　
    seg.setMaxIterations(100);             // 最大的迭代的次数
    seg.setDistanceThreshold(0.02);        // 设置符合模型的内点阈值

    // -------------模型分割,直到剩余点云数量在30%以上,确保模型点云较好----------
    int i = 0, nr_points = (int)cloud_filtered->points.size(); // 下采样前点云数量
    while (cloud_filtered->points.size() > 0.3 * nr_points)

    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients); // 分割
        if (inliers->indices.size() == 0)
        {
            cout << "Could not estimate a planar model for the given dataset." << endl;
            break;
        }
        //---------------------------根据索引提取点云-------------------------------
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers); // 提取符合平面模型的内点
        extract.setNegative(false);
        //--------------------------平面模型内点------------------------------------
        extract.filter(*cloud_plane);
        cout << "平面模型: " << cloud_plane->points.size() << "个点." << endl;
        //-------------------移去平面局内点，提取剩余点云---------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f; // 剩余点云
    }

    // --------------平面上的点云团,　使用欧式聚类的算法对点云聚类分割----------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);               // 桌子平面上其他的点云
    vector<pcl::PointIndices> cluster_indices;         // 点云团索引
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // 欧式聚类对象
    ec.setClusterTolerance(0.02);                      // 设置近邻搜索的搜索半径为2cm（也即两个不同聚类团点之间的最小欧氏距离）
    ec.setMinClusterSize(100);                         // 设置一个聚类需要的最少的点数目为100
    ec.setMaxClusterSize(25000);                       // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree);                          // 设置点云的搜索机制
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_all(new pcl::PointCloud<pcl::PointXYZ>);
    //------------迭代访问点云索引cluster_indices,直到分割处所有聚类---------------
    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //获取每一个点云团的点

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << endl;
        stringstream ss;
        ss << "../data/segmentation/euclidean_" << j << ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
        j++;

        *cloud_cluster_all += *cloud_cluster;
    }
    cout << "所有点: " << cloud_cluster_all->points.size() << "个." << endl;
    if (cloud_cluster_all->points.size() > 0)
        pcl::io::savePCDFileASCII("../data/segmentation/euclidean.pcd", *cloud_cluster_all);
    return (0);
}

// RANSAC拟合平面
pcl::PointCloud<pcl::PointXYZ> RandomSampleConsensus(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size)
{
    //--------------------------RANSAC拟合平面--------------------------
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(size); //设置距离阈值，与平面距离小于0.1的点作为内点
    ransac.computeModel();             //执行模型估计
    //-------------------------根据索引提取内点--------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;   //存储内点索引的容器
    ransac.getInliers(inliers); //提取内点索引
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
    //----------------------------输出模型参数---------------------------
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);

    //-----------平面模型的系数 A,B,C,D-----------
    double A = coefficient[0];
    double B = coefficient[1];
    double C = coefficient[2];
    double D = coefficient[3];
    cout << "平面方程为：\n"
         << A << "x + " << B << "y + " << C << "z + " << D << " = 0" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
    vector<int> pointIdxVec;
    for (int i = 0; i < source_cloud->points.size(); ++i)
    {
        float x0 = source_cloud->points[i].x;
        float y0 = source_cloud->points[i].y;
        float z0 = source_cloud->points[i].z;

        float absDistance = fabs(A * x0 + B * y0 + C * z0 + D) / sqrt(A * A + B * B + C * C); //计算点到平面的距离

        if (absDistance < 0.1) //距离阈值
        {
            pointIdxVec.push_back(i);
        }
    }
    pcl::copyPointCloud(*source_cloud, pointIdxVec, *cloud0);
    return *cloud0;
}

//区域生长分割
int RegionGrowing(std::string path, int type)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //从.pcd文件加载云
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    std::cout << "PointCloud: " << cloud->points.size() << " data points." << std::endl;
    //创建直通滤波器的对象
    // pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::string out_path = "";
    switch (type - 1)
    {
    case 0:
    {
        // 统计滤波器
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);    //设置待滤波的点云
        sor.setMeanK(50);            //设置在进行统计时考虑查询点临近点数
        sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阀值
        sor.setNegative(false);
        sor.filter(*cloud_filtered);
        out_path = "../data/segmentation/RG_StatisticalOutlierRemoval.pcd";
        break;
    }
    case 1:
    {
        // 半径滤波器
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);       //设置输入点云
        outrem.setRadiusSearch(0.1);       //设置半径为0.5的范围内找临近点
        outrem.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除
        outrem.filter(*cloud_filtered);
        out_path = "../data/segmentation/RG_RadiusOutlierRemoval.pcd";
        break;
    }
    default:
    {
        cout << "将不进行过滤" << endl;
        out_path = "../data/segmentation/RG.pcd";
        cloud_filtered = cloud;
        break;
    }
    }

    //估计表面法线
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_filtered);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // 区域生长聚类
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(500);         //最小的聚类的点数
    reg.setMaxClusterSize(1000000);    //最大的聚类的点数
    reg.setSearchMethod(tree);         //搜索方式
    reg.setNumberOfNeighbours(100);     //设置搜索的邻域点的个数
    reg.setInputCloud(cloud_filtered); //输入过滤后点云
    // reg.setIndices(indices);                        //对索引数组中列出的那些点进行分段
    reg.setInputNormals(normals);                   //输入的法线
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); //设置平滑度
    reg.setCurvatureThreshold(1);                 //设置曲率的阀值
    // 获取聚类的结果，分割结果保存在点云索引的向量中
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    //保存点云
    pcl::PCDWriter writer;
    /*pcl::PointCloud<pcl::PointXYZ> mls_cloud, ransac_cloud;
    double size = 0, distance = 0;
    cout << "输入MLS平滑半径和RANSAC距离阈值：" << endl;
    cin >> size >> distance;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;

        // MLS平滑
        pcl::PointCloud<pcl::PointXYZ> mls_points;
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setComputeNormals(true);
        // Set parameters
        mls.setInputCloud(cloud_cluster);
        mls.setPolynomialOrder(1);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(size);
        // Reconstruct
        mls.process(mls_points);
        mls_cloud += mls_points;

        if (cloud_cluster->points.size() > 2000)
            //拟合平面
            ransac_cloud += RandomSampleConsensus(cloud, mls_points.makeShared(), distance);
        // ss << "../data/segmentation/RG_" << j << ".pcd";
        // writer.write(ss.str(), mls_points, false);
        // writer.write<pcl::PointXYZ>(ss.str(), mls_points, false); // 保存文件
        j++;
    }
    std::stringstream mls_path, ransac_path;
    //保存MLS
    mls_path << "../data/segmentation/RG_MLS_" << size << ".pcd";
    writer.write(mls_path.str(), mls_cloud, false);
    //保存ransac
    std::cout << "Ransac Cloud: " << ransac_cloud.points.size() << " points." << std::endl;
    ransac_path << "../data/segmentation/RG_RANSAC_" << distance << ".pcd";
    writer.write(ransac_path.str(), ransac_cloud, false);*/

    //可视化聚类的结果
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    if (colored_cloud->width * colored_cloud->height > 0)
    {
        writer.write(out_path, *colored_cloud, false); // 分割得到的平面
        std::cout << "保存完成！" << std::endl;
    }
    else
        std::cout << "点云数据为空！" << std::endl;
    return (0);
}

// RANSAC拟合平面
int RandomSampleConsensus(std::string path)
{
    //-----------------------------读取点云----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(path, *cloud) < 0)
    {
        PCL_ERROR("点云读取失败！\n");
        return -1;
    }
    //--------------------------RANSAC拟合平面--------------------------
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(0.01); //设置距离阈值，与平面距离小于0.1的点作为内点
    ransac.computeModel();             //执行模型估计
    //-------------------------根据索引提取内点--------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;   //存储内点索引的容器
    ransac.getInliers(inliers); //提取内点索引
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
    //----------------------------输出模型参数---------------------------
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);
    cout << "平面方程为：\n"
         << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
         << coefficient[3] << " = 0" << endl;
    //-----------------------------结果可视化----------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}

int GetRandomNumber()
{
    int RandomNumber;
    RandomNumber = rand() % (256) + 0; // 0到255之间选择颜色
    //生成其他范围的数字：RandomNumber = rand() % (b-a+1) + a;
    return RandomNumber;
}

int RANSACPlane(std::string path)
{
    // ----------------------------加载点云-----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
    {
        PCL_ERROR("读取源标点云失败 \n");
        return (-1);
    }
    cout << "从点云中读取 " << cloud->size() << " 个点" << endl;

    //第一步：定义输入的原始数据以及分割获得的点、平面系数coefficients、存储内点的索引集合对象inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segment(new pcl::PointCloud<pcl::PointXYZ>); //创建分割对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);                   //模型系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                                  //索引列表
    pcl::SACSegmentation<pcl::PointXYZ> seg;                                                //分割对象

    pcl::ExtractIndices<pcl::PointXYZ> extract; //提取器

    int n_piece = 12; //需要探测的面的个数
    //第三步：使用RANSAC获取点数最多的面
    for (int i = 0; i < n_piece; i++)
    {
        seg.setOptimizeCoefficients(true);     //使用内部点重新估算模型参数
        seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型
        seg.setMethodType(pcl::SAC_RANSAC);    //设置随机采样一致性方法类型
        seg.setDistanceThreshold(0.01);        //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        //提取探测出来的平面
        extract.filter(*planar_segment);
        // planar_segment为该次探测出来的面片，可以单独进行保存，此处省略

        //剔除探测出的平面，在剩余点中继续探测平面
        extract.setNegative(true);
        extract.filter(*cloud);

        int R = GetRandomNumber();
        int G = GetRandomNumber();
        int B = GetRandomNumber();
        stringstream ss;
        // ----------------------------------------分割结果分类保存------------------------------------------
        ss << "../data/segmentation/Plane_" << i + 1 << ".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *planar_segment);
        cout << "第[" << i + 1 << "]块点云保存完毕！" << endl;
    }

    return (0);
}

void segmentationTest()
{
    std::cout << "--------------分割测试-------------" << std::endl;
    cout << "1.平面的分割" << endl;
    cout << "2.圆柱体模型的分割" << endl;
    cout << "3.欧式聚类分割" << endl;
    cout << "4.区域生长分割" << endl;
    cout << "5.RANSAC拟合平面" << endl;
    cout << "6.RANSAC多平面拟合" << endl;
    std::cout << "--------------分割测试-------------" << std::endl;
    int type = 0;
    cin >> type;
    std::cout << "输入pcd文件路径：" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (type - 1)
    {
    case 0:
        PlaneModel(path);
        break;
    case 1:
        CylinderMmodel(path);
        break;
    case 2:
        EuclideanCluster(path);
        break;
    case 3:
    {
        cout << "选择过滤条件：1.统计滤波器 2. 半径滤波 0.不进行过滤" << endl;
        int a = 0;
        cin >> a;
        RegionGrowing(path, a);
        break;
    }
    case 4:
        RandomSampleConsensus(path);
        break;
    case 5:
        RANSACPlane(path);
        break;
    default:
        cout << "编号输入错误" << endl;
        break;
    }
}