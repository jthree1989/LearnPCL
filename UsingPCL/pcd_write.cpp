#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = 100;
	cloud.height = 100;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (auto w = 0; w < cloud.width; ++w)
	  for(auto h = 0; h < cloud.height; ++h)
	{
		cloud.points[w * cloud.width + h].x = w;
		cloud.points[w * cloud.width + h].y = h;
		cloud.points[w * cloud.width + h].z = 255;
	}

	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	return 	system("pause");
}
