#include <vector>
#include <pcl\io\pcd_io.h>  
#include <pcl\io\ply_io.h>
#include <pcl\point_types.h>  
#include <pcl\filters\voxel_grid.h>
#include <pcl\visualization\cloud_viewer.h>   
#include <opencv2\core\core.hpp>  
#include <opencv2\contrib\contrib.hpp>
#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\imgproc\imgproc.hpp>  


using namespace std;
using namespace pcl;

int main()
{

#pragma region Region_1

	//string dir_path = "F:\\AppData\\vs2015\\PCL\\NumberCloud\\NumberCloud\\number_Image\\";
	//cv::Directory dir;
	//vector<string> number_path = dir.GetListFiles(dir_path, "*.png", true);

	//// 循环处理图像
	//for(int k =5;k<number_path.size();k++)
	//{
	//	cv::Mat srcImage = cv::imread(number_path.at(k));
	//	cv::Mat grayImage;
	//	cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
	//	cv::Mat Binary_Image;
	//	cv::threshold(grayImage, Binary_Image, 50, 255, cv::THRESH_BINARY_INV);
	//	// opencv 中的 sum 函数是对各个通道分别进行求和操作
	//	cv::Scalar s = cv::sum(Binary_Image);
	//	// 第0通道得到的和除以 255 的结果也就是有意义的点
	//	unsigned int image_point_size = s[0] / 255;

	//	// 再对图像进行遍历，如果是黑色区域，保留横纵坐标
	//	// 然后在放入 PCL 对相应点云进行赋值并保存的操作

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr number_PC(new pcl::PointCloud<pcl::PointXYZ>);

	//	// Fill in the cloud data  
	//	// height = 1 表示的是无序点云
	//	number_PC->width = image_point_size * 15;
	//	number_PC->height = 1;
	//	number_PC->points.resize(number_PC->width * number_PC->height);
	//	for (size_t i = 0; i < number_PC->points.size();)
	//	{
	//		for (size_t m = 0; m < 15; m++)		// m遍历的是 z 方向的坐标
	//		{
	//			// 内层遍历图像所有像素，如果不是 0 将坐标进行存储
	//			for (int a = 0; a < Binary_Image.rows; a = a++)
	//			{
	//				for (int b = 0; b < Binary_Image.cols; b = b++)
	//				{
	//					cout << "正在进行第 "<<k+1<<" 张图像的第 " << m+1 << " 层，" << a << " 行 " << b << " 列，" ;
	//					cout << "总进度正在完成 " << k+1 << "/10 "<< endl;
	//					if (Binary_Image.at<uchar>(a, b) != 0)
	//					{
	//						number_PC->points[i].x = b;
	//						number_PC->points[i].y = a;
	//						number_PC->points[i].z = m;
	//						i++;
	//					}
	//					else
	//						continue;
	//				}
	//			}



	//		}
	//	}

	//	// 显示点云
	//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL"));
	//	viewer->setBackgroundColor(255, 255, 255);
	//	pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ>color(number_PC);
	//	viewer->addPointCloud<pcl::PointXYZ>(number_PC, color, "src_ruler");
	//	string save_pathh = "F:\\AppData\\vs2015\\PCL\\NumberCloud\\NumberCloud\\number_PointCloud\\";
	//	pcl::io::savePLYFileASCII(save_pathh + to_string(k) + ".ply", *number_PC); //将点云保存到PCD文件中







	//	//int a = 0;
	//}

#pragma endregion Region_1

	// 读取点云并显示
	pcl::PointCloud<pcl::PointXYZ>::Ptr number_PC(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile("8.ply", *number_PC);

	pcl::PointCloud<pcl::PointXYZ>::Ptr number_PC1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("ruler_filtered_8 - Cloud.pcd", *number_PC1);
	cout << number_PC1->points.size() << endl;

	// 获取到 RGB 信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr number_RGB_XYZ(new pcl::PointCloud<pcl::PointXYZRGB>);
	number_RGB_XYZ->width = number_PC->width;
	number_RGB_XYZ->height = number_PC->height;
	number_RGB_XYZ->resize(number_PC->width * number_PC->height);

	// 将所有点改为蓝色
	for (size_t i = 0; i < number_RGB_XYZ->size(); i++)
	{
		number_RGB_XYZ->points[i].x = number_PC->points[i].x;
		number_RGB_XYZ->points[i].y = number_PC->points[i].y;
		number_RGB_XYZ->points[i].z = number_PC->points[i].z;
		//pPointsRGB->points[i].b = 255;
	}

	// 显示点云
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("PCL"));
	//viewer1->setBackgroundColor(255, 255, 255);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color1(number_RGB_XYZ);
	//viewer1->addPointCloud<pcl::PointXYZRGB>(number_RGB_XYZ, color1, "cube2");
	//while (!viewer1->wasStopped())
	//{
	//	viewer1->spinOnce();
	//}

	cout << number_RGB_XYZ->points.size() << endl;
	// 对 number_RGB_XYZ 进行下采样操作
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr number_RGB_XYZ_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);// 存储采样后的点云
	VoxelGrid<PointXYZRGB> sor;		// 创建滤波器对象
	sor.setInputCloud(number_RGB_XYZ);		// 设置需要下采样的点云对象
	sor.setLeafSize(10.0f, 10.0f, 10.0f);	// 设置滤波时创建的体素体积为 1cm^3 的立方体
	sor.filter(*number_RGB_XYZ_filtered);
	cout << number_RGB_XYZ_filtered->points.size() << endl;

	// 显示点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("PCL1"));
	viewer2->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color2(number_RGB_XYZ);
	viewer2->addPointCloud<pcl::PointXYZRGB>(number_RGB_XYZ_filtered, color2, "cube2");
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce();
	}

	// 使用每个方向的最大最小尺度进行归一化
	double max_x = -10e6, max_y = -10e6, max_z = -10e6;
	double min_x = 10e6, min_y = 10e6, min_z = 10e6;
	for(size_t idx = 0;idx<number_RGB_XYZ_filtered->points.size();idx++)
	{
		if (number_RGB_XYZ_filtered->points[idx].x > max_x)
			max_x = number_RGB_XYZ_filtered->points[idx].x;

		if (number_RGB_XYZ_filtered->points[idx].y > max_y)
			max_y = number_RGB_XYZ_filtered->points[idx].y;

		if (number_RGB_XYZ_filtered->points[idx].z > max_z)
			max_z = number_RGB_XYZ_filtered->points[idx].z;

		if (number_RGB_XYZ_filtered->points[idx].x < min_x)
			min_x = number_RGB_XYZ_filtered->points[idx].x;

		if (number_RGB_XYZ_filtered->points[idx].y < min_y)
			min_y = number_RGB_XYZ_filtered->points[idx].y;

		if (number_RGB_XYZ_filtered->points[idx].z < min_z)
			min_z = number_RGB_XYZ_filtered->points[idx].z;
	}
	cout << "max_x: " << max_x << "\t" << "max_y: " << max_y << "\t" << "max_z: " << max_z << endl;
	cout << "min_x: " << min_x << "\t" << "min_y: " << min_y << "\t" << "min_z: " << min_z << endl;
	

	cv::waitKey();
	return 0;
}

