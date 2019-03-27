#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

using namespace std;
using namespace pcl;
int main(int argc, char** argv)
{
	srand(time(NULL));	//select system clock as the seed of random number generator

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	// generate point cloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); i++){
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	KdTreeFLANN<PointXYZ> kdtree;

	kdtree.setInputCloud(cloud);

	PointXYZ searchPoint;

	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// K nearest neighbor search
	int K = 10;

	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);

	cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
				 << " " << cloud->points[pointIdxNKNSearch[i]].y
				 << " " << cloud->points[pointIdxNKNSearch[i]].z
				 << " (distance: " << sqrtf(pointNKNSquaredDistance[i]) << ")" << endl;
	}

	// Neighbors within radius search

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	cout << "Neighbors within radius search at (" << searchPoint.x
		 << " " << searchPoint.y
		 << " " << searchPoint.z
		 << ") with radius=" << radius << endl;

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
				 << " " << cloud->points[pointIdxRadiusSearch[i]].y
				 << " " << cloud->points[pointIdxRadiusSearch[i]].z
				 << " (distance: " << sqrtf(pointRadiusSquaredDistance[i]) << ")" << endl;
	}
	system("pause");
	return 0;
}