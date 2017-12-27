#include <vector>
#include <pcl\io\pcd_io.h>  
#include <pcl\io\ply_io.h>
#include <pcl\point_types.h>  
#include <pcl\visualization\cloud_viewer.h>   
#include <opencv2\core\core.hpp>  
#include <opencv2\contrib\contrib.hpp>
#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\imgproc\imgproc.hpp>  


using namespace std;
using namespace pcl;

int main()
{
	string dir_path = "F:\\AppData\\vs2015\\PCL\\NumberCloud\\NumberCloud\\number_Image\\";
	cv::Directory dir;
	vector<string> number_path = dir.GetListFiles(dir_path, "*.png", true);

	// 循环处理图像
	for(int k =6;k<number_path.size();k++)
	{
		cv::Mat srcImage = cv::imread(number_path.at(k));
		cv::Mat grayImage;
		cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
		cv::Mat Binary_Image;
		cv::threshold(grayImage, Binary_Image, 50, 255, cv::THRESH_BINARY_INV);
		// opencv 中的 sum 函数是对各个通道分别进行求和操作
		cv::Scalar s = cv::sum(Binary_Image);
		// 第0通道得到的和除以 255 的结果也就是有意义的点
		unsigned int image_point_size = s[0] / 255;

		// 再对图像进行遍历，如果是黑色区域，保留横纵坐标
		// 然后在放入 PCL 对相应点云进行赋值并保存的操作

		pcl::PointCloud<pcl::PointXYZ>::Ptr number_PC(new pcl::PointCloud<pcl::PointXYZ>);

		// Fill in the cloud data  
		// height = 1 表示的是无序点云
		number_PC->width = image_point_size * 15;
		number_PC->height = 1;
		number_PC->points.resize(number_PC->width * number_PC->height);
		for (size_t i = 0; i < number_PC->points.size();)
		{
			for (size_t m = 0; m < 15; m++)		// m遍历的是 z 方向的坐标
			{
				// 内层遍历图像所有像素，如果不是 0 将坐标进行存储
				for (int a = 0; a < Binary_Image.rows; a = a++)
				{
					for (int b = 0; b < Binary_Image.cols; b = b++)
					{
						cout << "正在进行第 "<<k+1<<" 张图像的第 " << m+1 << " 层，" << a << " 行 " << b << " 列，" ;
						cout << "总进度正在完成 " << k+1 << "/10 "<< endl;
						if (Binary_Image.at<uchar>(a, b) != 0)
						{
							number_PC->points[i].x = b;
							number_PC->points[i].y = a;
							number_PC->points[i].z = m;
							i++;
						}
						else
							continue;
					}
				}



			}
		}

		// 显示点云
		/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL"));
		viewer->setBackgroundColor(255, 255, 255);
		pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZ>color(number_PC);
		viewer->addPointCloud<pcl::PointXYZ>(number_PC, color, "src_ruler");*/
		string save_pathh = "F:\\AppData\\vs2015\\PCL\\NumberCloud\\NumberCloud\\number_PointCloud\\";
		pcl::io::savePLYFileASCII(save_pathh + to_string(k) + ".ply", *number_PC); //将点云保存到PCD文件中







		//int a = 0;
	}




	cv::waitKey();
	return 0;
}