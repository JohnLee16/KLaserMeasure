#pragma once
#include <iostream>
#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#ifndef FORMATTRANS_H
#define FORMATTRANS_H
//struct LIN
//{
//	double number[6];
//};
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_input;
class FormatTrans
{
public:
	FormatTrans();
	~FormatTrans();
	int Num_Points(const char *filename);
	void Format_Transformation(PointCloud_input cloud,const char * path);
private:

};
#endif
