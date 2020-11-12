/*-------------------------------------------------------------------*\

  NAME
    pointCloudIO.cpp


  DESCRIPTION
    Loading and Saving pcd files

\*-------------------------------------------------------------------*/

#include "pointCloudIO.h"


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string fileName)
/*-------------------------------------------------------------------*\
  Load pointcloud from PCD File
\*-------------------------------------------------------------------*/
{
    typename pcl::PointCloud<PointT>::Ptr pointCloud (new pcl::PointCloud<PointT>);

    // Read from File
    if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1) 
    {
        PCL_ERROR("***ERROR: File read failed!\n");
        return NULL;
    }
    std::cerr << "Loaded " << pointCloud->points.size () << " points from " << fileName << std::endl;

    return pointCloud;
}


template<typename PointT>
void savePcd(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::string fileName)
/*-------------------------------------------------------------------*\
  Save pointcloud as PCD File
\*-------------------------------------------------------------------*/
{
    pcl::io::savePCDFileASCII(fileName, *pointCloud);
    std::cerr << "Saved " << pointCloud->points.size() << " point cloud points to "+fileName << std::endl;
}


template<typename PointT>
std::vector<boost::filesystem::path> streamPcd(std::string dataPath)
/*-------------------------------------------------------------------*\
  Stream PCD file names from a directory
\*-------------------------------------------------------------------*/
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files into accending order 
    sort(paths.begin(), paths.end());

    return paths;
}

