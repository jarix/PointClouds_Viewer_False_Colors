/*-------------------------------------------------------------------*\

  NAME
    pointCloudUtils.h


  DESCRIPTION
    Some handy Point Cloud Utilities

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/
#ifndef POINT_CLOUD_UTILS_H
#define POINT_CLOUD_UTILS_H

#include <iostream> 

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry> 

/*-------------------------------------------------------------------*\
   Get the point range of a Point Cloud
\*-------------------------------------------------------------------*/
void getPointCloudRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, pcl::PointXYZI &minPoint, pcl::PointXYZI &maxPoint, bool bPrint = false);

/*-------------------------------------------------------------------*\
   Get the range of Intensity values in a Point Cloud
\*-------------------------------------------------------------------*/
void getIntensityRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, float &minIntensity, float &maxIntensity, bool bPrint = false);


/*-------------------------------------------------------------------*\
   Return point cloud sensor origin as PointXYZ
\*-------------------------------------------------------------------*/
void getSensorOrigin(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, pcl::PointXYZ &origin, bool bPrint = false);

#endif