/*-------------------------------------------------------------------*\

  NAME
    pointCloudUtils.cpp


  DESCRIPTION
    Some handy Point Cloud Utilities

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/

#include "pointCloudUtils.h"



void getPointCloudRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, pcl::PointXYZI &minPoint, pcl::PointXYZI &maxPoint, bool bPrint)
/*-------------------------------------------------------------------*\
   Get the point range of a Point Cloud
\*-------------------------------------------------------------------*/
{
    pcl::getMinMax3D(*pointCloud, minPoint, maxPoint);

    if (bPrint) {
        std::cout << "Point Cloud Size: " << pointCloud->size() << " points" << std::endl;
        std::cout << "Point Cloud Width: " << pointCloud->width << " Height: " << pointCloud->height << std::endl;
        std::cout << "Point Cloud Min Point ( " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << ")" << std::endl;
        std::cout << "Point Cloud Max Point ( " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << ")" << std::endl;
    }
}



void getIntensityRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, float &minIntensity, float &maxIntensity, bool bPrint)
/*-------------------------------------------------------------------*\
   Get the range of Intensity values in a Point Cloud
\*-------------------------------------------------------------------*/
{
    float minI = pointCloud->points[0].intensity;
    float maxI = pointCloud->points[0].intensity;

    for (auto it = pointCloud->points.begin(); it != pointCloud->points.end(); it++)
    {
        if ((*it).intensity < minI) {
            minI = (*it).intensity;
        }
        if ((*it).intensity > maxI) {
            maxI = (*it).intensity;
        }
    }    

    minIntensity = minI;
    maxIntensity = maxI;

    if (bPrint) {
        std::cout << "Intensity Min = " << minIntensity << std::endl;
        std::cout << "Intensity Max = " << maxIntensity << std::endl;      
    }
}



void getSensorOrigin(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, pcl::PointXYZ &origin, bool bPrint)
/*-------------------------------------------------------------------*\
   Return point cloud sensor origin as PointXYZ
\*-------------------------------------------------------------------*/
{
    Eigen::Vector4f evOrigin = pointCloud->sensor_origin_;
    origin.x = evOrigin(0);
    origin.y = evOrigin(1);
    origin.z = evOrigin(2);

    if (bPrint) {
        std::cout << "Sensor Origin: ("  << origin.x << "," << origin.y << "," << origin.z << ")" << std::endl;
    }
}

