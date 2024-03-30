#include "process.h"

//ransac平面拟合
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf& coefficient, float distance)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(distance);	//平面保留距离
	ransac.computeModel();				//执行模型估计

	pcl::PointIndices::Ptr inliers_plane_ptr(new pcl::PointIndices);
	ransac.getInliers(inliers_plane_ptr->indices);
	ransac.getModelCoefficients(coefficient);

	//提取内点对应的索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane_ptr);
	extract.setNegative(false);//是否剔除平面：false保留平面、true剔除平面
	extract.filter(*cloud_plane);

	cout << "ransac平面拟合个数为: " << cloud->points.size() << endl;
	cout << "ransac平面拟合用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_plane;
}

//直通
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
	float min, float max, string axis, bool is_save)
{
	pcl::PassThrough<pcl::PointXYZ> pass;//设置滤波器对象
	//参数设置
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(min, max);//区间设置
	pass.setFilterLimitsNegative(is_save);//设置为保留还是去除（true是去除上述坐标范围内的点）

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	pass.filter(*cloud_out);

	return cloud_out;
}

//平面分割
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_clip(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
{
	pcl::console::TicToc time;
	time.tic();

	//分割
	pcl::PlaneClipper3D<pcl::PointXYZ> clipper(plane);
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	clipper.clipPointCloud3D(*src_cloud, indices->indices);

	//提取
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(src_cloud);
	extract.setIndices(indices);
	extract.setNegative(negative);
	extract.filter(*dst_cloud);

	cout << "平面分割个数为: " << src_cloud->points.size() << endl;
	cout << "平面分割用时： " << time.toc() / 1000 << " 秒" << endl;

	return dst_cloud;
}

//删除重复点
pcl::PointCloud<pcl::PointXYZ>::Ptr delete_repeat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();
	//---------------------------KD树半径搜索---------------------------------
	pcl::search::KdTree <pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	pcl::Indices pointIdxR;  // 保存每个近邻点的索引
	vector<float> Distance;  // 保存每个近邻点与查找点之间的欧式距离平方
	float radius = 0.000001; // 距离阈值，若两点之间的距离为0.000001则认为是重合点
	set<int> remove_index;
	//对cloud中的每个点与邻域内的点进行比较
	for (auto& poiont_i : *cloud)
	{
		if (tree.radiusSearch(poiont_i, radius, pointIdxR, Distance) > 0)
		{
			if (pointIdxR.size() != 1)
			{
				for (size_t i = 1; i < pointIdxR.size(); ++i)
				{
					remove_index.insert(pointIdxR[i]);
				}
			}
		}
	}
	//--------------------------获取重复点的索引-----------------------------
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	copy(remove_index.cbegin(), remove_index.cend(), back_inserter(outliners->indices));
	//----------------------提取重复点索引之外的点云-------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);// 设置为true则表示保存索引之外的点
	extract.filter(*cloud_filtered);

	cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
	cout << "删除的重复点的个数为:" << remove_index.size() << endl;
	cout << "去重之后点的个数为:" << cloud_filtered->points.size() << endl;
	cout << "删除的重复点用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_filtered;
}

//统计滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_sor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int num, double stdmt)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outrem;//创建统计滤波对象
	//参数设置
	outrem.setInputCloud(cloud_in);
	outrem.setMeanK(num);//附近邻近点数
	outrem.setStddevMulThresh(stdmt);//标准差系数
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	outrem.filter(*cloud_out);

	cout << "SOR统计滤波个数为: " << cloud_in->points.size() << endl;
	cout << "SOR统计滤波用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_out;
}

//均匀
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_uniform_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float leaf_size)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::UniformSampling<pcl::PointXYZ> unifm_smp;
	unifm_smp.setRadiusSearch(leaf_size);
	unifm_smp.setInputCloud(cloud_in);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	unifm_smp.filter(*cloud_out);

	cout << "均匀滤波个数为: " << cloud_in->points.size() << endl;
	cout << "均匀滤波用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_out;
}

//投影滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
	float paraA, float paraB, float paraC, float paraD)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = paraA;
	coefficients->values[1] = paraB;
	coefficients->values[2] = paraC;
	coefficients->values[3] = paraD;

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_in);
	proj.setModelCoefficients(coefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	proj.filter(*cloud_projected);

	cout << "投影滤波个数为: " << cloud_in->points.size() << endl;
	cout << "投影滤波用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_projected;

}


//投影滤波
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, Eigen::Vector4f& coefficient)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficient(0);
	coefficients->values[1] = coefficient(1);
	coefficients->values[2] = coefficient(2);
	coefficients->values[3] = coefficient(3);

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_in);
	proj.setModelCoefficients(coefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	proj.filter(*cloud_projected);

	cout << "投影滤波个数为: " << cloud_in->points.size() << endl;
	cout << "投影滤波用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_projected;

}

// 计算法线
pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//------------------计算法线----------------------
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	n.setNumberOfThreads(10);//设置openMP的线程数
	//n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
	//n.setRadiusSearch(0.03);//半径搜素
	n.compute(*normals);//开始进行法向计算

	cout << "计算法线个数为: " << cloud->points.size() << endl;
	cout << "计算法线用时： " << time.toc() / 1000 << " 秒" << endl;

	return normals;
}

//计算alpha shapes平面点云边界特征提取
pcl::PointCloud<pcl::PointXYZ>::Ptr shapes_boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud); // 输入点云为投影后的点云
	chull.setAlpha(0.1);        // 设置alpha值为0.1
	chull.reconstruct(*cloud_hull);

	cout << "提取边界点个数为: " << cloud_hull->points.size() << endl;
	cout << "提取边界点用时： " << time.toc() / 1000 << " 秒" << endl;
	//-----------------结果显示---------------------
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

	// int v1(0), v2(0);
	// viewer->setWindowName("alpha_shapes提取边界");
	// viewer->setBackgroundColor(0, 0, 0);
	// viewer->addPointCloud(cloud, "cloud");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	// viewer->addPointCloud(cloud_hull, "cloud_boundary");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce(1000);
	// }

	return cloud_hull;
}

//计算平面方程
Eigen::MatrixXf getPlane() {
	pcl::console::TicToc time;
	time.tic();

	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("../data/io/plane.pcd", *cloud_1);//加载平面1
	pcl::io::loadPCDFile("../data/io/plane2.pcd", *cloud_2);//加载侧面2
	pcl::io::loadPCDFile("../data/io/plane3.pcd", *cloud_3);//加载侧面3
	pcl::io::loadPCDFile("../data/io/plane4.pcd", *cloud_4);//加载侧面4
	pcl::io::loadPCDFile("../data/io/plane5.pcd", *cloud_5);//加载侧面5

	Eigen::VectorXf coefficient;
	Eigen::VectorXf coefficient2;
	Eigen::VectorXf coefficient3;
	Eigen::VectorXf coefficient4;
	Eigen::VectorXf coefficient5;
	Eigen::VectorXf coefficient6;

	//平面分割，获取平面方程：上底面、侧面平面方程
	auto cloud_plane = pcl_ransac_plane(cloud_1, coefficient, 0.05);
	auto cloud_plane2 = pcl_ransac_plane(cloud_2, coefficient2, 0.05);
	auto cloud_plane3 = pcl_ransac_plane(cloud_3, coefficient3, 0.05);
	auto cloud_plane4 = pcl_ransac_plane(cloud_4, coefficient4, 0.05);
	auto cloud_plane5 = pcl_ransac_plane(cloud_5, coefficient5, 0.05);
	//保存下平面
	coefficient6 = coefficient;
	coefficient6(3) -= 3;

	cout << "计算平面方程用时： " << time.toc() / 1000 << " 秒" << endl;

	//-----------平面模型的系数 A,B,C,D-----------
	cout << "上平面方程为：\n"
		<< coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + " << coefficient[3] << " = 0" << endl;
	cout << "下平面方程为：\n"
		<< coefficient[0] << "x + " << coefficient6[1] << "y + " << coefficient6[2] << "z + " << coefficient6[3] << " = 0" << endl;
	cout << "左平面方程为：\n"
		<< coefficient2[0] << "x + " << coefficient2[1] << "y + " << coefficient2[2] << "z + " << coefficient2[3] << " = 0" << endl;
	cout << "右平面方程为：\n"
		<< coefficient3[0] << "x + " << coefficient3[1] << "y + " << coefficient3[2] << "z + " << coefficient3[3] << " = 0" << endl;
	cout << "左斜平面方程为：\n"
		<< coefficient2[0] << "x + " << coefficient4[1] << "y + " << coefficient4[2] << "z + " << coefficient4[3] << " = 0" << endl;
	cout << "右斜面方程为：\n"
		<< coefficient3[0] << "x + " << coefficient5[1] << "y + " << coefficient5[2] << "z + " << coefficient5[3] << " = 0" << endl;

	//保存平面方程坐标
	Eigen::MatrixXf planes(6, 4);
	planes.row(0) = coefficient;
	planes.row(1) = coefficient6;
	planes.row(2) = coefficient2;
	planes.row(3) = coefficient3;
	planes.row(4) = coefficient4;
	planes.row(5) = coefficient5;
	ofstream out_txt_file;	// 定义/打开输出的txt文件
	out_txt_file.open("../data/process/plane_equation.txt", ios::out | ios::trunc);
	out_txt_file << planes << std::endl;//每次写完一个矩阵以后换行
	out_txt_file.close();	// 关闭文件
	return planes;
}

//PCL点云处理
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//获取平面方程
	Eigen::MatrixXf planes = getPlane();

	Eigen::VectorXf coefficient1 = planes.row(0);//上
	Eigen::VectorXf coefficient2 = planes.row(2);//左
	Eigen::VectorXf coefficient3 = planes.row(3);//右
	Eigen::VectorXf coefficient4 = planes.row(4);//左斜
	Eigen::VectorXf coefficient5 = planes.row(5);//右斜
	Eigen::VectorXf coefficient6 = planes.row(1);//下

	//进行平面分割
	coefficient1(3) -= 3;
	auto cloud_seg = plane_clip(cloud, coefficient1, true);

	coefficient1(3) += 2;
	auto cloud_seg1 = plane_clip(cloud_seg, coefficient1, true); //上平面

	coefficient2(3) += 0.3;
	auto cloud_seg2 = plane_clip(cloud_seg, coefficient2, false); //左平面

	coefficient3(3) += 0.3;
	auto cloud_seg3 = plane_clip(cloud_seg, coefficient3, false); //右平面

	//下平面
	auto cloud_pro = pcl_filter_projection(cloud_seg, coefficient6(0), coefficient6(1), coefficient6(2), coefficient6(3));
	auto cloud_rst = *cloud_seg1 + *cloud_seg2 + *cloud_seg3 + *cloud_pro;
	//--------------------------------------------------------------------
	// 删除重复点
	auto cloud_delete = delete_repeat(cloud_rst.makeShared());
	//统计滤波——去除噪声
	auto cloud_sor = pcl_filter_sor(cloud_delete, 50, 2);
	//均匀滤波——降采样
	auto cloud_uniform = pcl_filter_uniform_sample(cloud_sor, 0.15);

	cout << "点云处理用时： " << time.toc() / 1000 << " 秒" << endl;

	return cloud_uniform;
}

void cloudProcess()
{
	pcl::console::TicToc time;
	time.tic();

	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../data/io/160m_clean.pcd", *cloud);//整体点云 
	//获取平面方程
	Eigen::MatrixXf planes = getPlane();

	Eigen::VectorXf coefficient1 = planes.row(0);//上
	Eigen::VectorXf coefficient2 = planes.row(2);//左
	Eigen::VectorXf coefficient3 = planes.row(3);//右
	Eigen::VectorXf coefficient4 = planes.row(4);//左斜
	Eigen::VectorXf coefficient5 = planes.row(5);//右斜
	Eigen::VectorXf coefficient6 = planes.row(1);//下

	//进行平面分割
	coefficient1(3) -= 3;
	auto cloud_seg = plane_clip(cloud, coefficient1, true);

	coefficient1(3) += 2;
	auto cloud_seg1 = plane_clip(cloud_seg, coefficient1, true); //上平面

	coefficient2(3) += 0.3;
	auto cloud_seg2 = plane_clip(cloud_seg, coefficient2, false); //左平面

	coefficient3(3) += 0.3;
	auto cloud_seg3 = plane_clip(cloud_seg, coefficient3, false); //右平面

	//下平面
	auto cloud_pro = pcl_filter_projection(cloud_seg, coefficient6(0), coefficient6(1), coefficient6(2), coefficient6(3));

	auto cloud_rst = *cloud_seg1 + *cloud_seg2 + *cloud_seg3 + *cloud_pro;

	//--------------------------------------------------------------------
	// 删除重复点
	auto cloud_delete = delete_repeat(cloud_rst.makeShared());

	//统计滤波——去除噪声
	auto cloud_sor = pcl_filter_sor(cloud_delete, 50, 2);
	saveCloud(cloud_sor, "../data/process/cloud_sor.pcd");
	pcl::io::savePLYFile("../data/process/cloud_sor.ply", *cloud_sor);

	//均匀滤波——降采样
	auto cloud_uniform = pcl_filter_uniform_sample(cloud_sor, 0.15);
	saveCloud(cloud_uniform, "../data/process/cloud_uniform.pcd");
	pcl::io::savePLYFile("../data/process/cloud.ply", *cloud_uniform);

	cout << "点云处理用时： " << time.toc() / 1000 << " 秒" << endl;

}

//平面切割
void cloudBoundaryClip(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f coefficients[], string out_file) 
{
	cout << "Size points: " << cloud->points.size() << endl;
	float d = 1e-5;//范围阈值
	coefficients[0](3) += d;
	auto cloud_seg1 = plane_clip(cloud, coefficients[0], false);//0上
	coefficients[1](3) -= d;
	auto cloud_seg2 = plane_clip(cloud_seg1, coefficients[1], true);//1下
	coefficients[2](3) -= d;
	auto cloud_seg3 = plane_clip(cloud_seg2, coefficients[2], true);//2左
	coefficients[3](3) -= d;
	auto cloud_seg4 = plane_clip(cloud_seg3, coefficients[3], true);// 3右
	coefficients[4](3) -= d;
	auto cloud_seg5 = plane_clip(cloud_seg4, coefficients[4], true);//4左斜
	coefficients[5](3) += d;
	auto cloud_seg6 = plane_clip(cloud_seg5, coefficients[5], false);//5右斜
	saveCloud(cloud_seg6, out_file);
}

//计算过滤后的各面投影并提取边界
void cloudProBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//读取平面参数
	Eigen::Vector4f coefficients[6] = { };//定义一个4*3的矩阵，用于存放数据
	ifstream infile;//定义读取文件流，相对于程序来说是in
	infile.open("../data/process/plane_equation.txt");//打开文件
	for (int i = 0; i < 6; i++)//定义行循环
	{
		for (int j = 0; j < 4; j++)//定义列循环
		{
			infile >> coefficients[i][j];//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
		}
	}
	infile.close();//读取完成之后关闭文件
	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_pro;
	cout << "Size points: " << cloud->points.size() << endl;
	Eigen::Vector4f coefficient = coefficients[0];//上
	//上平面
	coefficient(3) -= 0.5;
	cloud_seg = plane_clip(cloud, coefficient, true);
	coefficient(3) += 0.5;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	auto boundary = *shapes_boundary(cloud_seg); //提取边界
	//下平面
	coefficient = coefficients[1];
	coefficient(3) += 0.01;
	cloud_seg = plane_clip(cloud, coefficient, false);
	coefficient(3) -= 0.01;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	boundary += *shapes_boundary(cloud_seg); //提取边界
	//左平面
	coefficient = coefficients[2];
	coefficient(3) += 0.3;
	cloud_seg = plane_clip(cloud, coefficient, false);
	coefficient(3) -= 0.3;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	boundary += *shapes_boundary(cloud_seg); //提取边界
	//右平面
	coefficient = coefficients[3];
	coefficient(3) += 0.3;
	cloud_seg = plane_clip(cloud, coefficient, false);
	coefficient(3) -= 0.3;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	boundary += *shapes_boundary(cloud_seg); //提取边界
	//左斜平面
	coefficient = coefficients[4];
	coefficient(3) += 0.5;
	cloud_seg = plane_clip(cloud, coefficient, false);
	coefficient(3) -= 0.5;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	boundary += *shapes_boundary(cloud_seg); //提取边界
	//右斜平面
	coefficient = coefficients[5];
	coefficient(3) -= 0.5;
	cloud_seg = plane_clip(cloud, coefficient, true);
	coefficient(3) += 0.5;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	boundary += *shapes_boundary(cloud_seg); //提取边界
	//整体保存
	cloud_pro = *delete_repeat(cloud_pro.makeShared());	// 删除重复点
	cloudBoundaryClip(cloud_pro.makeShared(), coefficients, "../data/process/cloud_seg_clip.pcd");
	saveCloud(cloud_pro, "../data/process/cloud_seg.pcd");
	cloudBoundaryClip(boundary.makeShared(), coefficients, "../data/process/boundary_clip.pcd");
	saveCloud(boundary, "../data/process/boundary_hull.pcd");
}

void cloudProBoundary(string path)
{
	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(path, *cloud);//整体点云
	cloudProBoundary(cloud);
}

//计算前后面
pcl::PointCloud<pcl::PointXYZ>::Ptr aroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//读取平面参数
	Eigen::Vector4f coefficients[8] = { };//定义一个4*3的矩阵，用于存放数据
	ifstream infile;//定义读取文件流，相对于程序来说是in
	infile.open("../data/process/plane_equation.txt");//打开文件
	for (int i = 0; i < 8; i++)//定义行循环
	{
		for (int j = 0; j < 4; j++)//定义列循环
		{
			infile >> coefficients[i][j];//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行
		}
	}
	infile.close();//读取完成之后关闭文件
	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud_pro;
	cout << "Size points: " << cloud->points.size() << endl;
	Eigen::Vector4f coefficient = coefficients[6];//上
	//前平面
	coefficient(3) -= 0.5;
	cloud_seg = plane_clip(cloud, coefficient, true);
	coefficient(3) += 0.5;
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	//后平面
	coefficient = coefficients[7];
	cloud_seg = pcl_filter_projection(cloud_seg, coefficient);
	cloud_pro += *cloud_seg;
	//整体保存
	return cloud_pro.makeShared();
}

void aroundPlane()
{
	//声明变量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pro(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("../data/process/boundary_clip.pcd", *cloud);//整体点云
	cloud_pro = aroundPlane(cloud);
	//整体保存
	saveCloud(cloud_pro, "../data/process/aroundPlane.pcd");
}

void processTest() {
	std::cout << "--------------点云处理-------------" << std::endl;
	std::cout << "0.拟合平面处理" << std::endl;
	std::cout << "1.提取点云边界" << std::endl;
	std::cout << "2.提取前后面" << std::endl;
	std::cout << "--------------点云处理-------------" << std::endl;
	int type = 0;
	std::cin >> type;
	switch (type)
	{
	case 0:
		cloudProcess();
		break;
	case 1: {
		std::cout << "输入pcd文件路径:" << std::endl;
		std::string path = "";
		std::cin >> path;
		cloudProBoundary("../data/process/cloud_sor.pcd");
		break;
	}
	case 2:
		aroundPlane();
		break;
	default:
		std::cout << "编号输入错误!" << std::endl;
		break;
	}
}

