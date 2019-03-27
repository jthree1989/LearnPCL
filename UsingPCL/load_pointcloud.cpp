#include <iostream>
#include <string>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
template<typename T = PointXYZ> bool loadPCD(string filename,PointCloud<T>& cloud);
int main(int argc, char** argv)
{
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
	if (loadPCD<PointXYZRGB>(argv[1], *cloud)){
#if 0
		// Visualization
		visualization::PCLVisualizer viewer("Load PointCloud example");
		// Define R,G,B colors for the point cloud
		visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler(cloud, 255, 255, 255);
		// We add the point cloud to the viewer and pass the color handler
		viewer.addPointCloud<pcl::PointXYZRGB>(cloud, source_cloud_color_handler, "original_cloud");
		while (!viewer.wasStopped()){
			viewer.spinOnce();
		}
#else
		// Visualization
		visualization::CloudViewer viewer("Load PointCloud example");
		//viewer.showCloud(source_cloud);
		viewer.showCloud(cloud);
		while (!viewer.wasStopped())
		{
		}
#endif
	}
	return 0;
}

template<typename T>
bool loadPCD(string filename, PointCloud<T>& cloud)
{
#if 0
	if (-1 == io::loadPCDFile<T>(filename, cloud)){
		PCL_ERROR("Couldn't read %s file\n",filename);
		return false;
	}
#else
	PCLPointCloud2 cloud_blob;
	if (-1 == io::loadPCDFile(filename, cloud_blob)){
		PCL_ERROR("Couldn't read %s file\n", filename);
		return false;
	}
	else
		fromPCLPointCloud2(cloud_blob, cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
#endif
	return true;
}