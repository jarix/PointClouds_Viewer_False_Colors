/*-------------------------------------------------------------------*\

  NAME
    pointCloudIO.h


  DESCRIPTION
    Loading and Saving pcd files

\*-------------------------------------------------------------------*/
#ifndef POINT_CLOUD_IO_H
#define POINT_CLOUD_IO_H

#include <iostream> 
#include <string>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


/*-------------------------------------------------------------------*\
  Load pointcloud from PCD File
\*-------------------------------------------------------------------*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string fileName);

/*-------------------------------------------------------------------*\
  Save pointcloud as PCD File
\*-------------------------------------------------------------------*/
template<typename PointT>
void savePcd(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string fileName);

/*-------------------------------------------------------------------*\
  Stream PCD file names from a directory
\*-------------------------------------------------------------------*/
template<typename PointT>
std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

#endif