cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(UsingPCL)
find_package(PCL 1.8.0 REQUIRED)
#find_package(Qt5Widgets REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
SET(CMAKE_CXX_FLAGS "-std=c++11")
# write to pcd file 
add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
set_target_properties(pcd_write_test
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
# transform cloud with a transformation matrix
add_executable(matrix_transform matrix_transform.cpp)
target_link_libraries(matrix_transform ${PCL_LIBRARIES})

set_target_properties(matrix_transform
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
# normal estimation for cloud
#add_executable(normal_estimation normal_estimation.cpp)
#target_link_libraries(normal_estimation ${PCL_LIBRARIES})
#set_target_properties(normal_estimation
#	PROPERTIES
#	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#)
# load point cloud from pcd file
add_executable(load_pointcloud load_pointcloud.cpp)
target_link_libraries(load_pointcloud ${PCL_LIBRARIES})
set_target_properties(load_pointcloud
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
# test OpenNI grabber framework in PCL
add_executable(test_opennigrabber test_opennigrabber.cpp)
target_link_libraries(test_opennigrabber ${PCL_LIBRARIES})
set_target_properties(load_pointcloud
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)
# kdtree search example
#add_executable(kdtree_search kdtree_search.cpp)
#target_link_libraries(kdtree_search ${PCL_LIBRARIES})
#set_target_properties(kdtree_search
#	PROPERTIES
#	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
#)

# octree compression example
add_executable(point_cloud_compression  point_cloud_compression.cpp)
target_link_libraries(point_cloud_compression ${PCL_LIBRARIES})
set_target_properties(point_cloud_compression
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

# octree search example
add_executable(octree_search octree_search.cpp)
target_link_libraries(octree_search ${PCL_LIBRARIES})
set_target_properties(octree_search
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

add_executable (bspline_fitting bspline_fitting.cpp)
target_link_libraries (bspline_fitting ${PCL_LIBRARIES})
set_target_properties(bspline_fitting
	PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})
set_target_properties(resampling
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

add_executable (upsampling upsampling.cpp)
target_link_libraries (upsampling ${PCL_LIBRARIES})
set_target_properties(upsampling
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

add_executable(concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})
set_target_properties(concatenate_clouds
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin"
)

