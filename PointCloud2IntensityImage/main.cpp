#include "cloud2Figure.h"

int main()
{
	// ����ǿ��ͼ�ֱ���
	const double resolution = 0.05; 
	// ǿ��ֵ����ϵ��
	const double intensity_coeffocoent = 10; 
	// �����ļ�·��
	const std::string las_path = "D:/Document/sunquan/Code/PointCloud2Image/PointCloud2Image/demo.las";

	// ԭʼ����
	pcXYZIPtr cloud(new pcXYZI); 
	// ��������
	pcXYZIPtr gcloud(new pcXYZI);
	// ��������ȥ���
	pcXYZIPtr gcloud_filter(new pcXYZI);

	// ��ȡԭʼ��������  ת���������

	cloud2Figure* Cf = new cloud2Figure();
	Bounds bound_3d_temp;

	if (!Cf->readLasFile(las_path, cloud, bound_3d_temp))
	{
		return 0; // ��ȡlas�ļ�����) 
	}
	//// ��ȡ�߽� �Լ���������ת��
	//Cf->getBoundsOfCloud(cloud, bound_3d_temp);
	// ��ȡ�������
	Cf->extractGroundPoints(cloud, gcloud, bound_3d_temp);
	// ǿ��ֱͨ�˲�
	Cf->removalOutlinesPoints(gcloud, gcloud_filter, bound_3d_temp, intensity_coeffocoent);
	// ͶӰ����ͼ��
	cv::Mat imgI = Cf->pointCloud2imgI(gcloud_filter, resolution);

	cv::imwrite("intensity", imgI);
}