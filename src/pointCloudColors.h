/*-------------------------------------------------------------------*\

  NAME
    pointCloudColors.h


  DESCRIPTION
    Colorize Point Cloud based on intensity using OpenCV Colormaps

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/
#ifndef POINT_CLOUD_COLORS_H
#define POINT_CLOUD_COLORS_H

#include <iostream> 
#include <string>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


/*-------------------------------------------------------------------*\
  Colorize Point Cloud in false color based on intensity 
\*-------------------------------------------------------------------*/
void colorizeIntensityPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudI,   // XYZI Input Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,  // XYZRGB Point Cloud with colorw
    int colorMap,                                       // OpenCV ColorMap to use
    bool bDebugPrint = false
);

#endif