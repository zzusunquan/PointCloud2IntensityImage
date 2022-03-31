#include "cloud2Figure.h"

int main()
{
	// 生成强度图分辨率
	const double resolution = 0.05; 
	// 强度值过滤系数
	const double intensity_coeffocoent = 10; 
	// 点云文件路径
	const std::string las_path = "D:/Document/sunquan/Code/PointCloud2Image/PointCloud2Image/demo.las";

	// 原始点云
	pcXYZIPtr cloud(new pcXYZI); 
	// 地面点点云
	pcXYZIPtr gcloud(new pcXYZI);
	// 地面点点云去噪后
	pcXYZIPtr gcloud_filter(new pcXYZI);

	// 读取原始点云数据  转成相对坐标

	cloud2Figure* Cf = new cloud2Figure();
	Bounds bound_3d_temp;

	if (!Cf->readLasFile(las_path, cloud, bound_3d_temp))
	{
		return 0; // 读取las文件函数) 
	}
	//// 获取边界 以及点云坐标转换
	//Cf->getBoundsOfCloud(cloud, bound_3d_temp);
	// 提取地面点云
	Cf->extractGroundPoints(cloud, gcloud, bound_3d_temp);
	// 强度直通滤波
	Cf->removalOutlinesPoints(gcloud, gcloud_filter, bound_3d_temp, intensity_coeffocoent);
	// 投影生成图像
	cv::Mat imgI = Cf->pointCloud2imgI(gcloud_filter, resolution);

	cv::imwrite("intensity", imgI);
}