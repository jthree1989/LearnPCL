#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <dirent.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
using namespace std;
using namespace pcl;

vector<string> readFiles(const string& folder, const regex& reg);

int main(int argc, char* argv[])
{
  // 1. read the ply file into the pcl library
  string plyFoder("data");
  regex pattern("box_\\d\\.ply");
  vector<string> plyFiles = readFiles(plyFoder, pattern);
  // 2. concatenate all the data into one
  PointCloud<PointXYZRGBNormal>::Ptr allinOne(new PointCloud<PointXYZRGBNormal>);
  for(auto file:plyFiles){
    // 2.1 read data from ply file
    PointCloud<PointXYZRGBNormal>::Ptr cloud (new PointCloud<PointXYZRGBNormal>);
    io::loadPLYFile(file, *cloud);
    *allinOne += *cloud;
  }
  // 3. save the final file
  string finalPly("final.ply");
  io::savePLYFile(finalPly, *allinOne);
  
  return 0;
}


vector<string> readFiles(const string& folder, const regex& reg)
{
  vector<string> files;
  // 1. read filenames from folder
  DIR *dir;
  struct dirent *ent;
  dir = opendir(folder.c_str());
  if(nullptr != dir)
  {
    while (nullptr != (ent = readdir (dir))) 
    {
      if(strcmp(ent->d_name,".") != 0 && strcmp(ent->d_name,"..") != 0){
	bool ret = regex_match(ent->d_name, reg);
	if(ret)
	  files.push_back(folder+"/"+string(ent->d_name));
      } 
    }
  }
  return files;
}