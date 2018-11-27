#include <iostream>                           //标准输入输出头文件申明
#include <pcl/io/io.h>                        //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                    //PCD文件读取
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>   //类cloud_viewer头文件申明
#include <pcl/visualization/image_viewer.h>
#include "PCL_processing.h"
using namespace std;
using namespace pcl;

PC_Processing::PC_Processing()
{
}

PC_Processing::~PC_Processing()
{
}
