#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

int main(int argc,char** argv)
{
    std::string src_file, dst_file;
    double searchR = 2.0, upSamplingR = 0.03, upSamplingstep = 0.02;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);    
    //pcl::PointCloud<pcl::PointNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointNormal>);  

    if (argc < 3)
    {
      std::cout << "Usage: resampling src_file dst_file" << std::endl;
      return -1;
    }
    src_file = argv[1];
    dst_file = argv[2];
    if(argc >= 4)
      searchR = atof(argv[3]);
    if(argc >= 5)
      upSamplingR = atof(argv[4]);
    if(argc >= 6)
      upSamplingstep = atof(argv[5]);
    pcl::io::loadPCDFile (src_file, *cloud);
    // setup filters
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    //pcl::MovingLeastSquares<pcl::PointXYZ,  pcl::PointNormal> filter;
    filter.setInputCloud(cloud);
    // setup search method
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    // Use all neighbors in a radius in cm.
    filter.setSearchRadius(searchR);
    // If true, the surface and normal are approximated using a polynomial estimation(if false, only a tangent one).
    //filter.setPolynomialFit(true);
    // We can tell the algorithm to also compute smoothed normals (optional).
    filter.setComputeNormals(true);
    // Set the order of the polynomial to be fit.
    filter.setPolynomialOrder(2);
#if 0
    // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    //filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    // 采样的半径是
    filter.setUpsamplingRadius(upSamplingR);
    // 采样步数的大小
    filter.setUpsamplingStepSize(upSamplingstep);
#endif
#if 0
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
    filter.setPointDensity(600);
#endif

#if 1
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
    filter.setDilationVoxelSize(upSamplingR);
    filter.setDilationIterations(upSamplingstep);
#endif

    filter.process(*filteredCloud);
    
      // Save output
    pcl::io::savePCDFile (dst_file, *filteredCloud);
}