#include "FormatTrans.h"
#include "PCL_processing.h"
#include <iostream>
#include <pcl/io/io.h>                        //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                    //PCD文件读取
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>   //类cloud_viewer头文件申明
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
//#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
//#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
//#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件


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
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");      //创建viewer对象
	//viewer.showCloud(cloud_view);

	/*pcl::io::loadPCDFile("Laser_left_01.pcd", *cloud_l);
	pcl::io::loadPCDFile("laser_right_01.pcd", *cloud_r);*/

	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_l(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_r(new pcl::PointCloud<pcl::PointXYZ>);
	/*直通滤波器*/
	pcl::PassThrough<pcl::PointXYZ> passthrough;
	passthrough.setInputCloud(cloud_view_l);//输入点云
	passthrough.setFilterFieldName("y");//对y轴进行操作
	passthrough.setFilterLimits(-15, 15);//设置直通滤波器操作范围
	passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
	passthrough.filter(*cloud_after_PassThrough_l);//执行滤波，过滤结果保存在 cloud_after_PassThrough

	passthrough.setInputCloud(cloud_view_r);
	passthrough.setFilterFieldName("y");//对y轴进行操作
	passthrough.setFilterLimits(-15, 15);//设置直通滤波器操作范围
	passthrough.setFilterLimitsNegative(false);//true表示保留范围内，false表示保留范围外
	passthrough.filter(*cloud_after_PassThrough_r);//执行滤波，过滤结果保存在 cloud_after_PassThrough

	/*方法二：体素滤波器实现下采样*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid_l(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid_r(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setInputCloud(cloud_after_PassThrough_l);//输入点云数据
	voxelgrid.setLeafSize(1.0f, 1.0f, 1.0f);//AABB长宽高
	voxelgrid.filter(*cloud_after_voxelgrid_l);

	voxelgrid.setInputCloud(cloud_after_PassThrough_r);//输入点云数据
	voxelgrid.setLeafSize(1.0f, 1.0f, 1.0f);//AABB长宽高
	voxelgrid.filter(*cloud_after_voxelgrid_r);

	std::cout << "体素化网格方法后点云数据点数：" << cloud_after_voxelgrid_l->points.size() << std::endl;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_after_voxelgrid_l, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid_l, source_color, to_string(1), v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "1");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();   //初始化相机参数

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color2(cloud_after_voxelgrid_r, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_after_voxelgrid_r, source_color2, to_string(2), v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "2");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);  //显示
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //随时间
	}

	system("pause");
	return 0;
}