cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(UsingPCL)
if(${CMAKE_VERSION} VERSION_GREATER "3.12") 
    cmake_policy(SET CMP0074 NEW)
endif()
find_package(PCL 1.8.0 REQUIRED)
if( PCL_FOUND )
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif(PCL_FOUND)    
set(CMAKE_CXX_FLAGS "-std=c++11")

# define function to build pcl example
function(build_pcl_example exampleName)
    add_executable(${exampleName} ${exampleName}.cpp)
    target_link_libraries(${exampleName} ${PCL_LIBRARIES})
    set_target_properties(${exampleName}
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    )
endfunction(build_pcl_example)

# write to pcd file 
option(build_pcd_write "Build pcd_write example" OFF)
if(build_pcd_write)
    build_pcl_example(pcd_write)
endif(build_pcd_write)

# transform cloud with a transformation matrix
option(build_matrix_transform "Build matrix_transform example" OFF)
if(build_matrix_transform)
    build_pcl_example(matrix_transform)
endif(build_matrix_transform)

# normal estimation for cloud
option(build_normal_estimation "Build normal_estimation example" OFF)
if(build_normal_estimation)
    build_pcl_example(normal_estimation)
endif(build_normal_estimation)

# load point cloud from pcd file
option(build_load_pointcloud "Build load_pointcloud example" OFF)
if(build_load_pointcloud)
    build_pcl_example(load_pointcloud)
endif(build_load_pointcloud)

# test OpenNI grabber framework in PCL
option(build_test_opennigrabber "Build test_opennigrabber example" OFF)
if(build_test_opennigrabber)
    build_pcl_example(test_opennigrabber)
endif(build_test_opennigrabber)

# kdtree search example
option(build_kdtree_search "Build kdtree_search example" OFF)
if(build_kdtree_search)
    build_pcl_example(kdtree_search)
endif(build_kdtree_search)

# octree compression example
option(build_point_cloud_compression "Build point_cloud_compression example" OFF)
if(build_point_cloud_compression)
    build_pcl_example(point_cloud_compression)
endif(build_point_cloud_compression)

# octree search example
option(build_octree_search "Build octree_search example" OFF)
if(build_octree_search)
    build_pcl_example(octree_search)
endif(build_octree_search)

# bspline fitting example 
option(build_bspline_fitting "Build bspline_fitting example" OFF)
if(build_bspline_fitting)
    build_pcl_example(bspline_fitting)
endif(build_bspline_fitting)

# resampling example
option(build_resampling "Build resampling example" OFF)
if(build_resampling)
    build_pcl_example(resampling)
endif(build_resampling)

# upsampling example
option(build_upsampling "Build upsampling example" OFF)
if(build_upsampling)
    build_pcl_example(upsampling)
endif(build_upsampling)

# concatenate clouds example
option(build_concatenate_clouds "Build concatenate_clouds example" OFF)
if(build_concatenate_clouds)
    build_pcl_example(concatenate_clouds)
endif(build_concatenate_clouds)

# register clouds example
option(build_register_clouds "Build register_clouds example" OFF)
if(build_register_clouds)
    build_pcl_example(register_clouds)
endif(build_register_clouds)

# pairwise incremental registration example
option(build_incremental_registration "Build incremental_registration example" ON)
if(build_incremental_registration)
    build_pcl_example(incremental_registration)
endif(build_incremental_registration)
