#include "projection.h"

//投影到XOY平面
int XYZPro(PointCloudT::Ptr cloud, char type)
{
    cout << "点云的点数：" << cloud->points.size() << endl;
    // ------------------------ 遍历点云，令某一维度坐标为常数 -----------------------
    int pt_num = (int)cloud->points.size();
    switch (type)
    {
    case 'x': {
        for (int i = 0; i < pt_num; i++)
        {
            cloud->points[i].x = 0;
        }
        break;
    }
    case 'y': {
        for (int i = 0; i < pt_num; i++)
        {
            cloud->points[i].y = 0;
        }
        break;
    }
    case 'z': {
        for (int i = 0; i < pt_num; i++)
        {
            cloud->points[i].z = 0;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}

int XOY(string path)
{
    //---------------------------------- 加载点云 ----------------------------------
    cout << "->正在加载点云..." << endl;
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile(path, *cloud) < 0)
    {
        PCL_ERROR("\a->点云文件不存在！\n");
        return -1;
    }
    XYZPro(cloud, 'z');
    saveCloud(cloud, "../data/projection/plane_xoy.pcd");
    return 0;
}

//投影到XOZ平面
int XOZ(string path)
{
    //---------------------------------- 加载点云 ----------------------------------
    std::cout << "->正在加载点云..." << std::endl;
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile(path, *cloud) < 0)
    {
        PCL_ERROR("\a->点云文件不存在！\n");
        return -1;
    }
    XYZPro(cloud, 'y');
    saveCloud(cloud, "../data/projection/plane_xoz.pcd");
    return 0;
}

//投影到YOZ平面
int YOZ(string path)
{
    //---------------------------------- 加载点云 ----------------------------------
    std::cout << "->正在加载点云..." << std::endl;
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile(path, *cloud) < 0)
    {
        PCL_ERROR("\a->点云文件不存在！\n");
        return -1;
    }
    XYZPro(cloud, 'x');
    saveCloud(cloud, "../data/projection/plane_yoz.pcd");
    return 0;
}

//投影到指定平面
PointCloudT::Ptr setPlane(PointCloudT::Ptr cloud, Eigen::Vector4f plane)
{
    cout << "->加载点云的点数：" << cloud->points.size() << endl;
    //================================== 加载点云 ==================================

    //------------------- 创建平面模型 ax + by + cz + d = 0 ------------------------
    cout << "平面模型为：" << plane(0) << "x + " << plane(1) << "y + " << plane(2) << "z + " << plane(3) << " = 0:" << endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = plane(0);
    coefficients->values[1] = plane(1);
    coefficients->values[2] = plane(2);
    coefficients->values[3] = plane(3);
    //=================== 创建平面模型 ax + by + cz + d = 0 ========================

    //-------------------------------- 执行投影滤波 --------------------------------
    PointCloudT::Ptr cloud_projected(new PointCloudT);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    //================================ 执行投影滤波 ================================

    return cloud_projected;
}

int setPlane(string path)
{
    //---------------------------------- 加载点云 ----------------------------------
    cout << "->正在加载点云..." << endl;
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile(path, *cloud) < 0)
    {
        PCL_ERROR("\a->点云文件不存在！\n");
        return -1;
    }
    cout << "->加载点云的点数：" << cloud->points.size() << endl;
    //================================== 加载点云 ==================================

    //------------------- 创建平面模型 ax + by + cz + d = 0 ------------------------
    Eigen::Vector4f plane;
    cout << "创建平面模型 ax + by + cz + d = 0:" << endl;
    plane << 0.921625,0.385017,-0.025622,-44.051205;
    cout << "平面模型为：" << plane(0) << "x + " << plane(1) << "y + " << plane(2) << "z + " << plane(3) << " = 0:" << endl;
    PointCloudT::Ptr cloud_projected(new PointCloudT);
    cloud_projected = setPlane(cloud, plane);
    //保存滤波点云
    saveCloud(cloud_projected, "../data/projection/plane_project.pcd");
    return 0;
}

//计算平面到XYZ平面的角度
int planeAngle(Eigen::Vector3f plane)
{
    Eigen::Vector3f xoy;
    xoy << 0, 0, 1;
    Eigen::Vector3f xoz;
    xoz << 0, 1, 0;
    Eigen::Vector3f yoz;
    yoz << 1, 0, 0;
    float angle;	//夹角
    angle = pcl::getAngle3D(xoy, plane);
    cout << "->angle(xoy弧度) = " << angle << endl;
    angle = pcl::getAngle3D(xoy, plane, true);
    cout << "->angle(xoy角度) = " << angle << endl;

    angle = pcl::getAngle3D(xoz, plane);
    cout << "->angle(xoz弧度) = " << angle << endl;
    angle = pcl::getAngle3D(xoz, plane, true);
    cout << "->angle(xoz角度) = " << angle << endl;

    angle = pcl::getAngle3D(yoz, plane);
    cout << "->angle(yoz弧度) = " << angle << endl;
    angle = pcl::getAngle3D(yoz, plane, true);
    cout << "->angle(yoz角度) = " << angle << endl;
    return 0;
}

int planeAngle()
{
    Eigen::Vector3f plane;
    plane << -0.044928, 0.041122, -0.998143;
    planeAngle(plane);
    return 0;
}

//绕xyz轴旋转
pcl::PointCloud<pcl::PointXYZ>::Ptr transformationAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f plane)
{
    Eigen::Vector3f xoy;
    xoy << 0, 0, 1;
    Eigen::Vector3f xoz;
    xoz << 0, 1, 0;
    Eigen::Vector3f yoz;
    yoz << 1, 0, 0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); //初始化变换矩阵为单位矩阵
    // 在 X 轴上定义一个 1米的平移.
    // transform.translation() << 1.0, 0.0, 0.0;

    // 国际单位制中，弧度是角的度量单位，Eigen中也是以弧度作为角度量单位，
    // 因此需要将角度值转换为弧度制 角度转弧度 pcl::deg2rad(angle_z)
    transform.rotate(Eigen::AngleAxisf(pcl::getAngle3D(yoz, plane), Eigen::Vector3f::UnitX()));//X轴上旋转
    transform.rotate(Eigen::AngleAxisf(pcl::getAngle3D(xoz, plane), Eigen::Vector3f::UnitY()));//Y轴上旋转
    transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));//Z轴上旋转
    // 打印变换矩阵
    cout << "变换矩阵为：\n"
        << transform.matrix() << endl;

    // 执行变换，并将结果保存在新创建的 transformed_cloud 中
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

int transformationAngle(string path, Eigen::Vector3f plane)
{
    // 加载点云数据文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(path, *cloud);
    // 执行变换，并将结果保存在新创建的 transformed_cloud 中
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    transformed_cloud = transformationAngle(cloud, plane);
    saveCloud(transformed_cloud, "../data/projection/transformationAngle.pcd");
    return 0;
}

//变换矩阵变换点云
pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform)
{
    // 打印变换矩阵
    cout << "变换矩阵为：\n"
        << transform.matrix() << endl;
    // 执行变换，并将结果保存在新创建的 transformed_cloud 中
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

int transformationMatrix(string path, Eigen::Matrix4f transform)
{
    // 加载点云数据文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(path, *cloud);
    // 执行变换，并将结果保存在新创建的 transformed_cloud 中
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    transformed_cloud = transformationMatrix(cloud, transform);
    saveCloud(transformed_cloud, "../data/projection/transformationMatrix.pcd");
    return 0;
}

//求刚性变换矩阵
void rigid_transform_3D()
{
    vector<vector<double>> A, B;
    B = { {-0.044928, 0.041122, -0.998143} };
    A = { {0,0,1} };
    //double centroid_A, centroid_B;
    double sumAX = 0, sumAY = 0, sumAZ = 0;
    double sumBX = 0, sumBY = 0, sumBZ = 0;
    double mean_AX = 0, mean_AY = 0, mean_AZ = 0;
    double mean_BX = 0, mean_BY = 0, mean_BZ = 0;
    double AAX = 0, AAY = 0, AAZ = 0;
    double BBX = 0, BBY = 0, BBZ = 0;
    Eigen::Matrix< double, Eigen::Dynamic, 3> matrixA, matrixB;
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic> matrixAtrans ;
    Eigen::Matrix< double, 3, 3> matrixTrans;
    if(A.size() == B.size())
    {
        int size = A.size();
        Eigen::Vector3d centroid_A, centroid_B;
        for(int i = 0; i < size; i++)
        {
            sumAX += A[i][0];
            sumAY += A[i][1];
            sumAZ += A[i][2];
            sumBX += B[i][0];
            sumBY += B[i][1];
            sumBZ += B[i][2];
        }
        mean_AX = sumAX / size;
        mean_AY = sumAY / size;
        mean_AZ = sumAZ / size;
        mean_BX = sumBX / size;
        mean_BY = sumBY / size;
        mean_BZ = sumBZ / size;
 
        centroid_A << mean_AX, mean_AY, mean_AZ;
        centroid_B << mean_BX, mean_BY, mean_BZ;
 
        //矩阵A 的每列元素 减 该列的平均数
        for(int j = 0; j < size; j++)
        {
            AAX = A[j][0] - mean_AX;
            AAY = A[j][1] - mean_AY;
            AAZ = A[j][2] - mean_AZ;
            BBX = B[j][0] - mean_BX;
            BBY = B[j][1] - mean_BY;
            BBZ = B[j][2] - mean_BZ;
            matrixA.resize(size, 3);
            matrixA(j,0) = AAX;
            matrixA(j,1) = AAY;
            matrixA(j,2) = AAZ;
            matrixB.resize(size, 3);
            matrixB(j,0) = BBX;
            matrixB(j,1) = BBY;
            matrixB(j,2) = BBZ;
        }
 
        matrixAtrans = matrixA.transpose();
        matrixTrans = matrixAtrans * matrixB;   
 
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrixTrans, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
        Eigen::Matrix3d S = U.inverse() * matrixTrans * V.transpose().inverse();
        Eigen::Matrix3d Vt = V.transpose();
        Eigen::Matrix3d R = V * U.transpose(); //刚性变换矩阵
        Eigen::Vector3d t;
        double RDet = R.determinant();
        if (RDet < 0)
        {
            cout << "Reflection detected" << endl;
            Vt(2,0) = Vt(2,0)*(-1);
            Vt(2,1) = Vt(2,1)*(-1);
            Vt(2,2) = Vt(2,2)*(-1);
            R = Vt.transpose() * U.transpose();  //刚性变换矩阵
        }
        t = -(R * centroid_A) + centroid_B; //位移量
        // 打印变换矩阵
        cout << "变换矩阵为R：\n"
            << R.matrix() << endl;
        cout << "变换矩阵为t：\n"
            << t.matrix() << endl;
    }
}

void projectionTest(){
    std::cout << "--------------投影变化-------------" << std::endl;
    std::cout << "1.投影到XOY面" << std::endl;
    std::cout << "2.投影到XOZ面" << std::endl;
    std::cout << "3.投影到YOZ面" << std::endl;
    std::cout << "4.投影到指定面" << std::endl;
    std::cout << "5.计算XYZ的角度" << std::endl;
    std::cout << "6.指定轴旋转" << std::endl; 
    std::cout << "7.矩阵变换" << std::endl;
    std::cout << "8.刚性变换矩阵" << std::endl;
    std::cout << "--------------投影变化-------------" << std::endl;
    int type = 0;
    std::cin >> type;
    std::cout << "输入pcd文件路径：" << std::endl;
    std::string path = "../data/160m_clean.pcd";
    std::cin >> path;
    switch (type - 1)
    {
    case 0:
        XOY(path);
        break;
    case 1:
        XOZ(path);
        break;
    case 2:
        YOZ(path);
        break;
    case 3:
        setPlane(path);
        break;
    case 4:
        planeAngle();
        break;
    case 5:{
        Eigen::Vector3f plane;
        plane << -0.044928, 0.041122, -0.998143;
        transformationAngle(path, plane);
        break;
    }
    case 6: {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform << -0.049666, -0.382369, 0.922674, 127.541,
            0.0519178, 0.921576, 0.384709, 21.9635,
            -0.997416, 0.0670101 - 0.0259192, -3.08086,
            -0, -0, -0, 1;
        transformationMatrix(path, transform);
        break;
    }
    case 7:
        rigid_transform_3D();
        break;
    default:
        std::cout << "编号输入错误!" << std::endl;
        break;
    }
}