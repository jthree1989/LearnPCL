#include <dirent.h>
#include <string>
#include <vector>
#include <regex>
#include <algorithm>
#include <iostream>
using namespace std;

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/point_representation.h>   
using namespace pcl;

// some definitions
using PointT = pcl::PointXYZRGBNormal;
using MyPointCloud = pcl::PointCloud<PointT>;
using MyColorHandler = visualization::PointCloudColorHandlerRGBField <PointT>;
//using MyPointCloudNormal = pcl::PointCloud<pcl::PointNormal>;
//#define LOAD_DEBUG
// A structure to store the pointcloud data and its file name
struct MyPointData
{
    MyPointCloud::Ptr _cloud;
    string _name;
    
    MyPointData(): _cloud(new MyPointCloud){}
    
};
// PointRepresentation for ICP algorithm, convert PCL point struct to an n-dimensional vector
class MyPointRepresentation : public PointRepresentation<PointT>
{
    using pcl::PointRepresentation<PointT>::nr_dimensions_;
public:
    MyPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }
    
    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

/** \brief Load a set of pointcloud files that we want to register together
  * \param[in] folder   the path of floder that contains pointcloud files 
  * \param[in] reg      regex filter for files
  * \param[out] models  the resultant vector of point cloud datasets
  */

void loadData(const string& folder, const regex& reg, vector<MyPointData, Eigen::aligned_allocator<MyPointData>> &models)
{
    // 1. Read filenames from folder
    DIR *dir;
    struct dirent *ent;
    dir = opendir(folder.c_str());
    if(nullptr != dir){
        while (nullptr != (ent = readdir (dir))) {
            if(strcmp(ent->d_name,".") != 0 && strcmp(ent->d_name,"..") != 0){
                bool ret = regex_match(ent->d_name, reg);
                if(ret){
                    // 1.1 Find file with right suffix and load pointcloud data
                    MyPointData myData;
                    myData._name = folder+"/"+string(ent->d_name);                 
                    io::loadPLYFile(myData._name, *myData._cloud);
                    // 1.2 Remove NaN point in data
                    vector<int> indices;
                    removeNaNFromPointCloud(*myData._cloud, *myData._cloud, indices);
                    // 1.3 Convert millimeter to meter
                    for (auto i = 0; i < myData._cloud->points.size (); ++i){
                        myData._cloud->points[i].x /= 1000;
                        myData._cloud->points[i].y /= 1000;
                        myData._cloud->points[i].z /= 1000;
                    }
                    // 1.4 Push back data into models
                    models.push_back(myData);
                }
            } 
        }
    }
    // sort the models by file name
    sort(models.begin(), models.end(), [](MyPointData a, MyPointData b){
        string::size_type begin_a = a._name.find('_'), end_a = a._name.find('.'),
         begin_b = b._name.find('_'), end_b = b._name.find('.');
        int num_a = stoi(a._name.substr(begin_a + 1, end_a-begin_a + 1)),
            num_b = stoi(b._name.substr(begin_b + 1, end_b-begin_b + 1));
        return num_a < num_b;
    });
#ifdef LOAD_DEBUG
    for(auto data:models)
        cout << data._name << endl;
#endif   
}

/** \brief  Show source and target pointcloud in left view port
 *  \param[in] target       Target pointcloud
 *  \param[in] source       Source pointcloud
 *  \param[in] visualizer   PCL visualizer
 *  \param[in] viewPortID   ID of view port
 */
void showCloudsLeft(const MyPointCloud::Ptr& target, const MyPointCloud::Ptr& source, visualization::PCLVisualizer::Ptr& visualizer, int viewPortID)
{
    // 1. Remove previous pointcloud
    visualizer->removePointCloud("source");
    visualizer->removePointCloud("target");
    // 2. Set pointcloud color handler
    MyColorHandler targetColorHandler(target), sourceColorHandler(source);
    if(!targetColorHandler.isCapable())
        PCL_WARN ("Cannot create curvature color handler on target pointcloud!");
    if(!sourceColorHandler.isCapable())
        PCL_WARN ("Cannot create curvature color handler on source pointcloud!");
    // 3. Add source and target pointcloud to visualizer
    visualizer->addPointCloud<PointT>(target, targetColorHandler, "target", viewPortID);
    visualizer->addPointCloud<PointT>(source, sourceColorHandler, "source", viewPortID);
    // 4. Display  
    PCL_INFO ("Press q to begin the registration.\n");
    visualizer-> spin();
}

/** \brief Display source and target on the right viewport of the visualizer
 *  \param[in] target       Target pointcloud
 *  \param[in] source       Source pointcloud
 *  \param[in] visualizer   PCL visualizer
 *  \param[in] viewPortID   ID of view port
 */
void showCloudsRight(const MyPointCloud::Ptr target, const MyPointCloud::Ptr source, visualization::PCLVisualizer::Ptr& visualizer, int viewPortID)
{
    // 1. Remove previous pointcloud
    visualizer->removePointCloud("source");
    visualizer->removePointCloud("target");
    // 2. Set pointcloud color handler
    MyColorHandler targetColorHandler(target), sourceColorHandler(source);
    if(!targetColorHandler.isCapable())
        PCL_WARN ("Cannot create curvature color handler on target pointcloud!");
    if(!sourceColorHandler.isCapable())
        PCL_WARN ("Cannot create curvature color handler on source pointcloud!");
    // 3. Add source and target pointcloud to visualizer
    visualizer->addPointCloud<PointT>(target, targetColorHandler, "target", viewPortID);
    visualizer->addPointCloud<PointT>(source, sourceColorHandler, "source", viewPortID);
    // 4. Display  
    visualizer->spinOnce();
}

/** \brief Align a pair of PointCloud datasets and return the result
  * \param[in]  source      the source pointcloud
  * \param[in]  target      the target pointcloud
  * \param[out] output      the resultant aligned source PointCloud
  * \param[out] transform   the resultant transform between source and target
  * \param[in]  visualizer  PCL visualizer
  * \param[in]  viewPortID  ID of view port
  * \param[in]  downsample  if downsample is needed, set it true for large datasets
  * \param[in]  calNormals  if calculate normals from pointcloud, if pointcloud has normals, set it false
  */
void alignPair(const MyPointCloud::Ptr source, const MyPointCloud::Ptr target, MyPointCloud::Ptr output, Eigen::Matrix4f &final_transform, 
               visualization::PCLVisualizer::Ptr& visualizer, int viewPortID, bool downsample = false, bool calNormals = false)
{
    // 1. downsample the pointcloud if needed
    MyPointCloud::Ptr downsampledSource(new MyPointCloud), downsampledTarget(new MyPointCloud);
    if(downsample){
        // Define voxel grid filter to downsample pointcloud
        VoxelGrid<PointT> voxelGrid;
        float voxelSize = 0.05;        // Set the voxel size to 5 * 5 * 5 mm^3
        voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
        voxelGrid.setInputCloud(source);
        voxelGrid.filter(*downsampledSource);
        voxelGrid.setInputCloud(target);
        voxelGrid.filter(*downsampledTarget);
        
    }
    else{
        //downsampledSource = source;
        //downsampledTarget = target;
        copyPointCloud(*source, *downsampledSource);
        copyPointCloud(*target, *downsampledTarget);
    }
    // 2. Estimate normals if needed
    if(calNormals){
        //MyPointCloudNormal::Ptr sourceWithNormals(new MyPointCloudNormal), targetWithNormals(new MyPointCloudNormal);
        //PointCloud<PointXYZ>::Ptr sourceXYZ(new PointCloud<PointXYZ>), targetXYZ(new PointCloud<PointXYZ>);
        //copyPointCloud(*downsampledSource, *sourceXYZ);
        //copyPointCloud(*downsampledTarget, *targetXYZ);
        NormalEstimation<PointT, PointT> normalEstimation;
        search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT> ());
        normalEstimation.setSearchMethod(tree);
        normalEstimation.setKSearch(30);                    // Set the number of k nearest neighbors
        // Estimate normals of source pointcloud
        normalEstimation.setInputCloud(downsampledSource);
        normalEstimation.compute(*downsampledSource);
        // Estimate normals of target pointcloud
        normalEstimation.setInputCloud(downsampledTarget);
        normalEstimation.compute(*downsampledTarget);
    }
    // 3. Registeration
    // 3.1 Use nonlinear ICP algorithm
    IterativeClosestPointNonLinear<PointT, PointT> icp_nl;
    icp_nl.setTransformationEpsilon(1e-6);                  // Set the threhold for ICP converage
    icp_nl.setMaxCorrespondenceDistance(0.1);               // Set the maximum distance(0.1m) between two correspondences
    // 3.2 Set user-defined point representation 
    MyPointRepresentation point_representation;             // Instantiate our custom point representation (defined above) ...
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};                  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    point_representation.setRescaleValues (alpha);
    icp_nl.setPointRepresentation(boost::make_shared<const MyPointRepresentation> (point_representation));
    // 3.3 Set source and target pointcloud
    icp_nl.setInputSource(downsampledSource);
    icp_nl.setInputTarget(downsampledTarget);
    // 3.4 Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    MyPointCloud::Ptr icpResult = downsampledSource;
    icp_nl.setMaximumIterations (2);
    for(auto i = 0; i < 30; ++i){
        //PCL_INFO ("Iteration Nr. %d.\n", i);
        // 3.4.1 Save cloud for visualization purpose
        downsampledSource = icpResult;
        // 3.4.2 Estimate
        icp_nl.setInputSource(downsampledSource);
        icp_nl.align(*icpResult);
        cout << "has converged:" << icp_nl.hasConverged() << " score: " << icp_nl.getFitnessScore() << endl;
        cout << icp_nl.getFinalTransformation() << endl;
        // 3.4.3 Accumulate transformation between each iteration
        Ti = icp_nl.getFinalTransformation() * Ti;
        // 3.4.4 If the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing the maximal correspondence distance
        if (fabs ((icp_nl.getLastIncrementalTransformation () - prev).sum ()) < icp_nl.getTransformationEpsilon ())
            icp_nl.setMaxCorrespondenceDistance (icp_nl.getMaxCorrespondenceDistance () - 0.001);
        
        prev = icp_nl.getLastIncrementalTransformation ();
        // 3.4.5 Visualize current state
        showCloudsRight(downsampledTarget, downsampledSource, visualizer, viewPortID);
    }
    // 3.5 Get the transformation from target to source
    targetToSource = Ti.inverse();
    cout << "targetToSource: \n" << targetToSource << endl;
    // 3.6 Transform target back in source frame
    transformPointCloud (*target, *output, targetToSource);
    // 3.7 Display 
    visualizer->removePointCloud ("source");
    visualizer->removePointCloud ("target");
    visualization::PointCloudColorHandlerCustom<PointT> targetCloudHandler (output, 0, 255, 0);
    visualization::PointCloudColorHandlerCustom<PointT> sourceCloudHandler (source, 255, 0, 0);
    //MyColorHandler targetCloudHandler (output), sourceCloudHandler(source);
    visualizer->addPointCloud (output, targetCloudHandler, "target", viewPortID);
    visualizer->addPointCloud (source, sourceCloudHandler, "source", viewPortID);

	PCL_INFO ("Press q to continue the registration.\n");
    visualizer->spin ();
    visualizer->removePointCloud ("source"); 
    visualizer->removePointCloud ("target");

    //3.8 Add the source to the transformed target
    *output += *source;
    final_transform = targetToSource;
}

int main(int argc, char* argv[])
{
    // 1. Load all the pointcloud data from files
    vector<MyPointData, Eigen::aligned_allocator<MyPointData>> data;
    string plyFoder("data");
    regex pattern("face_\\d{1,2}\\.ply");
    loadData(plyFoder, pattern, data);
    // 2. Check if the data is valid
    if(data.empty()){
        PCL_ERROR("No pointcloud data loaded!");
        return -1;
    }
    // 3. Create visual tools for displaying pointcloud
    visualization::PCLVisualizer::Ptr pclVisualizer(new visualization::PCLVisualizer(argc, argv, "Incremental Registration Example"));
    int leftViewID, rightViewID;
    pclVisualizer->createViewPort(0.0, 0.0, 0.5, 1.0, leftViewID);
    pclVisualizer->createViewPort(0.5, 0.0, 1.0, 1.0, rightViewID);
    // 4. Process all the pointcloud
    MyPointCloud::Ptr result(new MyPointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    
    for(auto i = 1; i < data.size(); i++){
        // 4.1 Select two close pointcloud, and show them in left view port
        source = data[i-1]._cloud;
        target = data[i]._cloud;
        showCloudsLeft(source, target, pclVisualizer, leftViewID);
        // 4.2 Register these two pointcloud
        MyPointCloud::Ptr temp(new MyPointCloud);
        PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1]._name.c_str (), source->points.size (), data[i]._name.c_str (), target->points.size ());
        alignPair(source, target, temp, pairTransform, pclVisualizer, rightViewID);
        // 4.3 Transform current pair into the global transform
        transformPointCloud (*temp, *result, GlobalTransform);
        // 4.4 Update the global transform
        GlobalTransform = GlobalTransform * pairTransform;
        //save aligned pair, transformed into the first cloud's frame
        stringstream ss;
        ss <<"data/registration/"<< i << ".ply";
        io::savePLYFile(ss.str (), *result);
    }
    return 0;
}
