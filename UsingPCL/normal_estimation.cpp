#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

// this function displays the help
void showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}

// Estimate total PointCloud
template<typename T = PointXYZ,typename U = Normal>
void EstimateNormal(boost::shared_ptr< PointCloud<T> > source, boost::shared_ptr< PointCloud<U> > output){
	// create the normal estimation class, and pass the input dataset to it
	NormalEstimation<T, U> normalEst;
	normalEst.setInputCloud(source);

	// create an empty kdtree representation, and pass it to the normal estimation object.
	// its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<T>::Ptr kdTree(new pcl::search::KdTree<T>());
	normalEst.setSearchMethod(kdTree);

	// use all neighbors in a sphere of radius 3cm
	normalEst.setRadiusSearch(3);

	// compute result into output datasets
	normalEst.compute(*output);
}
// Estimate subset of PointCloud
template<typename T = PointXYZ, typename U = Normal>
void EstimateSubsetNormal(boost::shared_ptr<PointCloud<T>> source,
						  boost::shared_ptr<PointCloud<U>> output,
						  vector<int>& indice){
	// create the normal estimation class, and pass the input dataset to it
	NormalEstimation<T, U> normalEst;
	normalEst.setInputCloud(source);

	// pass the indices
	boost::shared_ptr<vector<int>> indiceptr(new vector<int>(indice));
	normalEst.setIndices(indiceptr);

	// create an empty kdtree representation, and pass it to the normal estimation object.
	// its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<T>::Ptr kdTree(new pcl::search::KdTree<T>());
	normalEst.setSearchMethod(kdTree);

	// use all neighbors in a sphere of radius 3cm
	normalEst.setRadiusSearch(3);

	// compute result into output datasets
	normalEst.compute(*output);
}
int main(int argc, char** argv)
{
	/*-- load pcd file --*/ 

	// show help
	if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
		return 0;
	}

	// parse filename from input command
	vector<int> filenames = console::parse_file_extension_argument(argc, argv, ".pcd");
	if (1 != filenames.size()){
		showHelp(argv[0]);
		return -1;
	}
	
	// load pcd file from filename
	PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(argv[filenames[0]], *sourceCloud) < 0){
		cout << "Error loading point cloud " << argv[filenames[0]] << endl;
		showHelp(argv[0]);
		return -1;
	}

	/*-- compute normal --*/
	// estimate total PointCloud normal 
	PointCloud<Normal>::Ptr normalCloud(new PointCloud<Normal>());
	EstimateNormal<PointXYZ,Normal>(sourceCloud,normalCloud);

	// estimate subset PointCloud normal
	// Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
	vector<int> indices(floor(sourceCloud->points.size() / 10));
	for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;
	PointCloud<Normal>::Ptr subsetNormalCloud(new PointCloud<Normal>());
	EstimateSubsetNormal<PointXYZ, Normal>(sourceCloud, subsetNormalCloud,indices);

	/*-- visualization --*/
	visualization::PCLVisualizer viewer("Normal Estimation Example");
	// add the point cloud to the viewer
	viewer.addPointCloudNormals<PointXYZ,Normal>(sourceCloud, normalCloud, 1, 1, "NormalCloud",0);
	// Display the visualizer until 'q' key is pressed
	while (!viewer.wasStopped()) { 
		viewer.spinOnce();
	}

	system("pause");
	return 0;
}