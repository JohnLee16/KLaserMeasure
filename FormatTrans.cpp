#include "FormatTrans.h"

using namespace std;

FormatTrans::FormatTrans()
{
}

FormatTrans::~FormatTrans()
{
}

int FormatTrans::Num_Points(const char * filename)
{
	int n = 0;
	int c = 0;
	FILE *fp;

	fp = fopen(filename, "r");
	do {
		c = fgetc(fp);
		if (c == '\n') {
			++n;
		}
	}
	while (c != EOF);

	fclose(fp);
	return n;
}

void FormatTrans::Format_Transformation(PointCloud_input cloud,const char * path)
{
	static int lrNr = 0;
	int n = 0;
	
	n = FormatTrans::Num_Points(path);

	cloud.width = n;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	double x, y, z;
	double Count_x = 0.0066, Count_z = 0;
	int i = 0;
	FILE *fp;
	fp = fopen(path, "r");

	while (i < n) {
		fscanf(fp, "%lf\n", &y);
		if (Count_x > 40)
		{
			Count_x = 0.00666;
			Count_z += 0.125;
		}
		if (y == 0)
		{
			x = 0;
			z = 0;
		}
		else
		{
			x = Count_x;
			z = Count_z;
		}
		//cout << x << " " << y << " " << z << endl;
		cloud.points[i].x = x;
		cloud.points[i].y = y;
		cloud.points[i].z = z;
		++i;
		Count_x += 0.00666;

	}

	fclose(fp);
	if (lrNr==0)
	{
		pcl::io::savePCDFileASCII("Laser_left_01.pcd", cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to Laser_left_01.pcd." << endl;
		lrNr++;
	}
	else
	{
		pcl::io::savePCDFileASCII("Laser_right_01.pcd", cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to Laser_right_01.pcd." << endl;
	}

	
}
