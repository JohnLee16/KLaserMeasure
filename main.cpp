#include "FormatTrans.h"
#include "PCL_processing.h"
#include <iostream>
#include <pcl/io/io.h>                        //I/O���ͷ�ļ�����
#include <pcl/io/pcd_io.h>                    //PCD�ļ���ȡ
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>   //��cloud_viewerͷ�ļ�����
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/passthrough.h>  //ֱͨ�˲���ͷ�ļ�
#include <pcl/filters/voxel_grid.h>  //�����˲���ͷ�ļ�
//#include <pcl/filters/statistical_outlier_removal.h>   //ͳ���˲���ͷ�ļ�
//#include <pcl/filters/conditional_removal.h>    //�����˲���ͷ�ļ�
//#include <pcl/filters/radius_outlier_removal.h>   //�뾶�˲���ͷ�ļ�


int main() {
	int laser_No = 2;
	while (laser_No > 0)
	{
		const char * path;
		if (laser_No==2)
			path = "D:\\WORK\\DATA\\PCD2\\test_l.txt";
		if (laser_No==1)
			path = "D:\\WORK\\DATA\\PCD2\\test_r.txt";
		FormatTrans format_t;
		PointCloud_input cloud;
		format_t.Format_Transformation(cloud, path);
		laser_No--;
	}
	

	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 1, 1, v1);

	pcl::PCLPointCloud2::Ptr cloud_l(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_r(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view_l(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view_r(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("Laser_left_01.pcd", *cloud_l);
	reader.read("laser_right_01.pcd", *cloud_r);
	pcl::fromPCLPointCloud2(*cloud_l, *cloud_view_l);
	pcl::fromPCLPointCloud2(*cloud_r, *cloud_view_r);
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");      //����viewer����
	//viewer.showCloud(cloud_view);

	/*pcl::io::loadPCDFile("Laser_left_01.pcd", *cloud_l);
	pcl::io::loadPCDFile("laser_right_01.pcd", *cloud_r);*/

	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_l(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_r(new pcl::PointCloud<pcl::PointXYZ>);
	/*ֱͨ�˲���*/
	pcl::PassThrough<pcl::PointXYZ> passthrough;
	passthrough.setInputCloud(cloud_view_l);//�������
	passthrough.setFilterFieldName("y");//��y����в���
	passthrough.setFilterLimits(-15, 15);//����ֱͨ�˲���������Χ
	passthrough.setFilterLimitsNegative(false);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
	passthrough.filter(*cloud_after_PassThrough_l);//ִ���˲������˽�������� cloud_after_PassThrough

	passthrough.setInputCloud(cloud_view_r);
	passthrough.setFilterFieldName("y");//��y����в���
	passthrough.setFilterLimits(-15, 15);//����ֱͨ�˲���������Χ
	passthrough.setFilterLimitsNegative(false);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
	passthrough.filter(*cloud_after_PassThrough_r);//ִ���˲������˽�������� cloud_after_PassThrough

	/*�������������˲���ʵ���²���*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid_l(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid_r(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(cloud_after_PassThrough_l);//�����������
	voxelgrid.setLeafSize(1.0f, 1.0f, 1.0f);//AABB�����
	voxelgrid.filter(*cloud_after_voxelgrid_l);

	voxelgrid.setInputCloud(cloud_after_PassThrough_r);//�����������
	voxelgrid.setLeafSize(1.0f, 1.0f, 1.0f);//AABB�����
	voxelgrid.filter(*cloud_after_voxelgrid_r);

	std::cout << "���ػ����񷽷���������ݵ�����" << cloud_after_voxelgrid_l->points.size() << std::endl;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_after_voxelgrid_l, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid_l, source_color, to_string(1), v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "1");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();   //��ʼ���������

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color2(cloud_after_voxelgrid_r, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid_r, source_color2, to_string(2), v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "2");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //��ʾ
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //��ʱ��
	}

	system("pause");
	return 0;
}