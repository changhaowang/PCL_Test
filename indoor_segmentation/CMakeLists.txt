cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Indoor_Segmentation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_generate_straight pcd_generate_straight.cpp)
add_executable (pcd_generate pcd_generate.cpp)
add_executable (Indoor_segmentation Indoor_segmentation.cpp)
#add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (pcd_generate ${PCL_LIBRARIES})
target_link_libraries (Indoor_segmentation ${PCL_LIBRARIES})
target_link_libraries (pcd_generate_straight ${PCL_LIBRARIES})

