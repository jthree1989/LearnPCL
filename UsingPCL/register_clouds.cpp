#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h> 

using namespace std;
using namespace pcl;

int main(int argc, char* argv[])
{
  // 1. read two pointclouds
  string plyFoder("data/"), targetName("face_1.ply"), sourceName("face_5.ply");
  PointCloud<PointXYZRGBNormal>::Ptr source(new PointCloud<PointXYZRGBNormal>);
  PointCloud<PointXYZRGBNormal>::Ptr target(new PointCloud<PointXYZRGBNormal>);
  PointCloud<PointXYZRGBNormal>::Ptr final(new PointCloud<PointXYZRGBNormal>);
  if(argc == 2) sourceName = string(argv[1]);
  if(argc == 3) {sourceName = string(argv[1]), targetName = string(argv[2]);}
  io::loadPLYFile(plyFoder + targetName, *target);
  io::loadPLYFile(plyFoder + sourceName, *source);
  // 2. calculate transformation between them, and register pointclouds
  IterativeClosestPoint<PointXYZRGBNormal, PointXYZRGBNormal> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.align(*final);
  *final += *target;
  cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
  cout << icp.getFinalTransformation() << endl;
  // 3. save the final file
  string finalPly("data/final_align.ply");
  io::savePLYFile(finalPly, *final);

  return 0;
}
