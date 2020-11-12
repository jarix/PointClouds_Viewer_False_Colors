cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(cyclone_intensity_viewer)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (pointCloudViewer src/pointCloudViewer.cpp src/pointCloudIO.cpp src/pointCloudUtils.cpp src/pointCloudColors.cpp)
target_link_libraries (pointCloudViewer ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

