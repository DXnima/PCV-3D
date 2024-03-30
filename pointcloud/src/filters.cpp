#include "filters.h"

//直通滤波器
pcl::PointCloud<pcl::PointXYZ>::Ptr PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    /************************************************************************************
     创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
     即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
     ***********************************************************************************/
     // 设置滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);          //设置输入点云
    pass.setFilterFieldName("z");       //设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits(0.0, 1.0);     //设置在过滤字段的范围
    pass.setFilterLimitsNegative(true); // true：滤波结果取反，被过滤的点  false: 过滤后的点
    pass.filter(*cloud_filtered);       //执行滤波，保存过滤结果在cloud_filtered

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    return cloud_filtered;
}

int PassThrough(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //点云对象的读取
    pcl::PCDReader reader;

    reader.read(path, *cloud); //读取点云到cloud中

    cloud_filtered = PassThrough(cloud);
   
    saveCloud(cloud_filtered, "../data/filter/PassThrough.pcd");

    return (0);
}

// VoxelGrid滤波器
//使用体素化网格方法实现下采样，即减少点的数量 减少点云数据，并同时保存点云的形状特征，在提高配准，曲面重建，形状识别等算法速度中非常实用
pcl::PCLPointCloud2::Ptr VoxelGrid(pcl::PCLPointCloud2::Ptr cloud, double size)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    /******************************************************************************
    创建一个voxel叶大小为1cm的pcl::VoxelGrid滤波器，
  **********************************************************************************/
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //创建滤波对象
    sor.setInputCloud(cloud);                //设置需要过滤的点云给滤波对象
    sor.setLeafSize(size, size, size);       //设置滤波时创建的体素体积为20cm的立方体
    sor.setFilterLimitsNegative(false);      // true：滤波结果取反，被过滤的点  false: 过滤后的点
    sor.filter(*cloud_filtered);             //执行滤波处理，存储输出

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

    return cloud_filtered;
}

int VoxelGrid(std::string path, double size)
{

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    //点云对象的读取
    pcl::PCDReader reader;

    reader.read(path, *cloud); //读取点云到cloud中

    cloud_filtered = VoxelGrid(cloud, size);
    saveCloud(cloud_filtered, "../data/filter/VoxelGrid.pcd");
    return (0);
}

//使用statisticalOutlierRemoval滤波器移除离群点\噪声点
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int number, double size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud(cloud);                          //设置待滤波的点云
    sor.setMeanK(number);                              //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(size);                      //设置判断是否为离群点的阀值
    sor.setNegative(false);                            // true：滤波结果取反，被过滤的点  false: 过滤后的点
    sor.filter(*cloud_filtered);                       //存储

    return cloud_filtered;
}

int StatisticalOutlierRemoval(std::string path, int number, double size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 定义读取对象
    pcl::PCDReader reader;
    // 读取点云文件
    reader.read<pcl::PointXYZ>(path, *cloud);
    cloud_filtered = StatisticalOutlierRemoval(cloud, number, size);
    saveCloud(cloud_filtered, "../data/filter/StatisticalOutlierRemoval.pcd");
    return (0);
}

//半径滤波 删除在输入点云一定范围内没有达到足够多近邻的所有数据点
pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r, int number)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    // 半径滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);            //设置输入点云
    outrem.setRadiusSearch(r);              //设置半径为0.5的范围内找临近点
    outrem.setMinNeighborsInRadius(number); //设置查询点的邻域点集数小于1的删除
    outrem.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

    return cloud_filtered;
}

int RadiusOutlierRemoval(std::string path, double r, int number)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 定义读取对象
    pcl::PCDReader reader;
    // 读取点云文件
    reader.read<pcl::PointXYZ>(path, *cloud);
    cloud_filtered = RadiusOutlierRemoval(cloud, r, number);
    saveCloud(cloud_filtered, "../data/filter/RadiusOutlierRemoval.pcd");
    return (0);
}

//均值下采样
pcl::PointCloud<pcl::PointXYZ>::Ptr UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double r)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    // 创建均匀采样（模板）类对象，点数据类型为 pcl::PointXYZRGB
    pcl::UniformSampling<pcl::PointXYZ> unisam;
    // 设置输入点云，注意：此处传入的是点云类对象的智能指针
    unisam.setInputCloud(cloud);
    // 设置采样半径，数值越大越稀疏
    unisam.setRadiusSearch(r);
    // 执行过滤，并带出处理后的点云数据，注意：此处传入的是点云类对象
    unisam.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    return cloud_filtered;
}

int UniformSampling(std::string path, double r)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0)
    {
        return -1;
    }
    cloud_filtered = UniformSampling(cloud, r);
    saveCloud(cloud_filtered, "../data/filter/UniformSampling.pcd");
    return (0);
}

//增采样
pcl::PointCloud<pcl::PointNormal> MovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double s_r, double u_r, double size)
{
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;
    /*
        // 滤波对象
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        //建立搜索对象
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
        filter.setSearchMethod(kdtree);
        //设置搜索邻域的半径为1cm=0.01
        filter.setSearchRadius(s_r);
        // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
        filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        // 采样的半径是
        filter.setUpsamplingRadius(u_r);
        // 采样步数的大小
        filter.setUpsamplingStepSize(size);
        filter.process(*cloud_filtered);
    */

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> cloud_filtered;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(s_r);     // 0.03 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
    mls.setPolynomialOrder(size); // 拟合曲线的阶数

    // Reconstruct
    mls.process(cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height
        << " data points (" << pcl::getFieldsList(cloud_filtered) << ")." << std::endl;

    return cloud_filtered;
}

int MovingLeastSquares(std::string path, double s_r, double u_r, double size)
{
    // 新建点云存储对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> cloud_filtered;

    // 读取文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0)
    {
        return -1;
    }
    cloud_filtered = MovingLeastSquares(cloud, s_r, u_r, size);
    saveCloud(cloud_filtered, "../data/filter/MovingLeastSquares.pcd");
    return (0);
}

//双边滤波 （耗时较长）
pcl::PointCloud<pcl::PointXYZI>::Ptr BilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 需要PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    // Apply the filter
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSearchMethod(tree1);
    fbf.setStdDev(0.1);
    fbf.setHalfSize(0.1);
    fbf.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    return cloud_filtered;
}

int BilateralFilter(std::string path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 需要PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // 定义读取对象
    pcl::PCDReader reader;
    // 读取点云文件
    reader.read<pcl::PointXYZI>(path, *cloud);
    cloud_filtered = BilateralFilter(cloud);
    saveCloud(cloud_filtered, "../data/filter/BilateralFilter.pcd");
    return (0);
}

//条件过滤
pcl::PointCloud<pcl::PointXYZRGB>::Ptr conditionalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int* rgb)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "before size: " << cloud->size() << std::endl;
    //创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>()); //创建条件定义对象

    // GT大于 EQ等于 LT小于 GE大于等于 LE小于等于
    //为条件定义对象添加比较算子 添加在rgb字段上小于（255，0，0）的比较算子
    std::uint32_t rgb_filter = ((std::uint32_t)rgb[0] << 16 | (std::uint32_t)rgb[1] << 8 | (std::uint32_t)rgb[2]);
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("rgb", pcl::ComparisonOps::LT, *reinterpret_cast<float*>(&rgb_filter))));
    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);    //输入点云
    condrem.setKeepOrganized(false); //设置保持点云的结构 点的数目没有减少，采用nan代替了
                                     // 设置是否保留滤波后删除的点，以保持点云的有序性，通过setuserFilterValue设置的值填充点云；或从点云中删除滤波后的点，从而改变其组织结构
                                     // 如果设置为true且不设置setUserFilterValue的值，则用nan填充点云
    // 执行滤波
    condrem.filter(*cloud_filtered); //条件用于建立滤波器
    std::cerr << "PointCloud after filtering: " << cloud_filtered->size() << std::endl;
    return (0);
}

int conditionalFilter(std::string path)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }
    int rgb[] = { 255,0,0 };
    cloud_filtered = conditionalFilter(cloud, rgb);
    saveCloud(cloud_filtered, "../data/filter/conditionalFilterRGB.pcd");
    return (0);
}

//按颜色过滤
int rgbFilter()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    //填充点云
    cloud->width = (std::uint32_t) 5;
    cloud->height = (std::uint32_t) 1;
    cloud->points.resize((cloud->width) * (cloud->height));

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f) / 1000;
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f) / 1000;
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f) / 1000;
        std::uint32_t rgb = ((std::uint32_t)110 << 16 | (std::uint32_t)110 << 8 | (std::uint32_t)110);
        cloud->points[i].rgb = *reinterpret_cast<float *>(&rgb);
    }
    std::uint32_t rgb = ((std::uint32_t)255 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)0);
    cloud->points[2].rgb = *reinterpret_cast<float *>(&rgb);
    std::uint32_t rgb1 = ((std::uint32_t)255 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)0);
    std::uint32_t rgb2 = ((std::uint32_t)0 << 16 | (std::uint32_t)255 << 8 | (std::uint32_t)0);
    std::uint32_t rgb3 = ((std::uint32_t)0 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)255);
    std::cerr << " 255 0 0:" << *reinterpret_cast<float *>(&rgb1) << std::endl;
    std::cerr << " 0 255 0:" << *reinterpret_cast<float *>(&rgb2) << std::endl;
    std::cerr << " 0 0 255:" << *reinterpret_cast<float *>(&rgb3) << std::endl;

    std::cerr << " FieldsList:" << pcl::getFieldsList(*cloud) << std::endl;
    //创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>()); //创建条件定义对象
    //为条件定义对象添加比较算子 过滤红色的点rgb(255 0 0)
    std::uint32_t rgb_filter = ((std::uint32_t)255 << 16 | (std::uint32_t)0 << 8 | (std::uint32_t)0);
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("rgb", pcl::ComparisonOps::LT, *reinterpret_cast<float *>(&rgb_filter)))); //添加在r字段上等于255的比较算子
    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);    //输入点云
    condrem.setKeepOrganized(false); //设置保持点云的结构
                                     // 设置是否保留滤波后删除的点，以保持点云的有序性，通过setuserFilterValue设置的值填充点云；或从点云中删除滤波后的点，从而改变其组织结构
                                     // 如果设置为true且不设置setUserFilterValue的值，则用nan填充点云

    // 执行滤波
    condrem.filter(*cloud_filtered); //大于0.0小于0.8这两个条件用于建立滤波器
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << " "
                  << (int)cloud->points[i].r << " "
                  << (int)cloud->points[i].g << " "
                  << (int)cloud->points[i].b << std::endl;

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " "
                  << cloud_filtered->points[i].y << " "
                  << cloud_filtered->points[i].z << " "
                  << (int)cloud_filtered->points[i].r << " "
                  << (int)cloud_filtered->points[i].g << " "
                  << (int)cloud_filtered->points[i].b << std::endl;
    return (0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GaussianKernel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "PointCloud before filtering: " << cloud_filtered->size() << std::endl;
    //-----------基于高斯核函数的卷积滤波实现------------------------
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
    kernel.setSigma(4);                    //高斯函数的标准方差，决定函数的宽度
    kernel.setThresholdRelativeToSigma(4); //设置相对Sigma参数的距离阈值
    kernel.setThreshold(0.05);             //设置距离阈值，若点间距离大于阈值则不予考虑
    cout << "Kernel made" << endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    cout << "KdTree made" << endl;

    //---------设置Convolution 相关参数---------------------------
    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
    convolution.setKernel(kernel); //设置卷积核
    convolution.setInputCloud(cloud);
    convolution.setNumberOfThreads(8);
    convolution.setSearchMethod(tree);
    convolution.setRadiusSearch(0.01);
    cout << "Convolution Start" << endl;

    convolution.convolve(*cloud_filtered);

    //------------- 保存点云-----------------------------
    std::cerr << "PointCloud after filtering: " << cloud_filtered->size() << std::endl;

    return cloud_filtered;
}

int GaussianKernel(std::string path)
{
    //------------------加载数据------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile(path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file pcd\n");
        return (-1);
    }
    cloud = GaussianKernel(cloud);
    //------------- 保存点云-----------------------------
    saveCloud(cloud, "../data/filter/GaussianKernel.pcd");
    return 0;
}

//获取斜率满足条件的点
int FilterAngle(std::string path, int angleA, int angleB) {

    //------------------加载数据------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file pcd\n");
        return (-1);
    }

    pcl::console::TicToc time;
    time.tic();
    
    //---------------------- 提取点云索引 -------------------
    cout << "->正在提取点云索引..." << endl;
    pcl::PointIndices indices;	///点云索引index数组
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        auto poiont_i = cloud->points[i];
        if (poiont_i.x < 0) {
            double k = poiont_i.y / poiont_i.x;
            // 提取XY斜率在指定角度范围内的
            if (tan(angleA * PI / 180) <= k && k <= tan(angleB * PI / 180))
            {
                indices.indices.push_back(i);
            }
        }
    }
    //--------------------------获取满足条件点的索引-----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_indice(new pcl::PointCloud<pcl::PointXYZ>);
    // 将自定义的indices数组进行智能指针的转换
    pcl::IndicesPtr index_ptr(new std::vector<int>(indices.indices));
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(index_ptr);
    extract.setNegative(false);	// 设置为false则表示保存索引之内的点
    extract.filter(*cloud_indice);

    cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
    cout << "满足条件点的个数为:" << index_ptr->size() << endl;
    cout << "用时： " << time.toc() / 1000 << " 秒" << endl;
    //计算最大最小点
    getMinMax3DXYZ(cloud_indice);
    saveCloud(cloud_indice,"../data/filter/Angle.pcd");
    //--------------------可视化点云索引提取结果----------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("点云索引提取结果"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "insides cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_indice, "outsides cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "insides cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "outsides cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

void filterTest()
{
    std::cout << "--------------过滤测试-------------" << std::endl;
    std::cout << "1.直通滤波PassThrough" << std::endl;
    std::cout << "2.体素滤波下采样VoxelGrid" << std::endl;
    std::cout << "3.移除离散点StatisticalOutlierRemoval" << std::endl;
    std::cout << "4.半径滤波RadiusOutlierRemoval" << std::endl;
    std::cout << "5.均值采用UniformSampling" << std::endl;
    std::cout << "6.增采样MovingLeastSquares" << std::endl;
    std::cout << "7.双边滤波BilateralFilter" << std::endl;
    std::cout << "8.高斯滤波GaussianKernel" << std::endl;
    std::cout << "9.获取斜率满足条件的点" << std::endl;
    std::cout << "--------------过滤测试-------------" << std::endl;
    int type = 0;
    std::cin >> type;
    std::cout << "输入pcd文件路径：" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (type - 1)
    {
    case 0:
        PassThrough(path);
        break;
    case 1:
    {
        std::cout << "输入体素体积:" << std::endl;
        double size = 0.5f;
        std::cin >> size;
        VoxelGrid(path, size);
        break;
    }
    case 2:
    {
        std::cout << "设置查询点临近点数和离群点的阀值：" << std::endl;
        int number = 50;
        double size = 1.0;
        std::cin >> number >> size;
        StatisticalOutlierRemoval(path, number, size);
        break;
    }
    case 3:
    {
        std::cout << "设置半径和邻域点集数阀值：" << std::endl;
        double r = 0.1;
        int number = 2;
        std::cin >> r >> number;
        RadiusOutlierRemoval(path, r, number);
        break;
    }
    case 4:
    {
        std::cout << "设置采样半径：" << std::endl;
        double r = 0.01f;
        std::cin >> r;
        UniformSampling(path, r);
        break;
    }
    case 5:
    {
        std::cout << "设置搜索邻域的半径、采样半径、采样大小：" << std::endl;
        double s_r = 0.03, u_r = 0.03, size = 0.02;
        std::cin >> s_r >> u_r >> size;
        MovingLeastSquares(path, s_r, u_r, size);
        break;
    }
    case 6:
        BilateralFilter(path);
        break;
    case 7:
        GaussianKernel(path);
        break;
    case 8: 
    {
        std::cout << "设置角度范围（如 30 -30）：" << std::endl;
        int a = 30, b = -30;
        std::cin >> a >> b;
        FilterAngle(path, a, b);
        break;
    }
    default:
        std::cout << "编号输入错误" << std::endl;
        break;
    }
}