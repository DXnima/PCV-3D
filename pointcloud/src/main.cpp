#include "main.h"

int main()
{
	//设置控制台编码为 UTF-8
	SetConsoleOutputCP(65001);
	int i = 1;
	std::cout << "--------------主菜单-------------" << std::endl;
	std::cout << "1.IO显示与保存" << std::endl;
	std::cout << "2.Filter过滤" << std::endl;
	std::cout << "3.点云分割" << std::endl;
	std::cout << "4.表面重建" << std::endl;
	std::cout << "5.模型贴图" << std::endl;
	std::cout << "6.旋转变换" << std::endl;
	std::cout << "7.提取边界" << std::endl;
	std::cout << "8.点云处理" << std::endl;
	std::cout << "9.几何计算" << std::endl;
	std::cout << "10.包围盒" << std::endl;
	std::cout << "11.综合处理" << std::endl;
	std::cout << "0.退出" << std::endl;
	std::cout << "--------------主菜单-------------" << std::endl;
	while (i != 0)
	{
		std::cin >> i;
		switch (i)
		{
		case 0:
			return 0;
		case 1:
			pcdTest();
			break;
		case 2:
			filterTest();
			break;
		case 3:
			segmentationTest();
			break;
		case 4:
			surfaceTest();
			break;
		case 5:
			textureTest();
			break;
		case 6:
			projectionTest();
			break;
		case 7:
			boundaryTest();
			break;
		case 8:
			processTest();
			break;
		case 9:
			calculateTest();
			break;
		case 10:
			bboxTest();
			break;
		case 11:
			collectPCL();
			break;
		default:
			std::cout << "编号输入错误 重新输入！" << std::endl;
			break;
		}
		std::cout << "--------------主菜单-------------" << std::endl;
		std::cout << "1.IO显示与保存" << std::endl;
		std::cout << "2.Filter过滤" << std::endl;
		std::cout << "3.点云分割" << std::endl;
		std::cout << "4.表面重建" << std::endl;
		std::cout << "5.模型贴图" << std::endl;
		std::cout << "6.旋转变换" << std::endl;
		std::cout << "7.提取边界" << std::endl;
		std::cout << "8.点云处理" << std::endl;
		std::cout << "9.几何计算" << std::endl;
		std::cout << "10.包围盒" << std::endl;
		std::cout << "11.综合处理" << std::endl;
		std::cout << "0.退出" << std::endl;
		std::cout << "--------------主菜单-------------" << std::endl;
	}
	return 0;
}