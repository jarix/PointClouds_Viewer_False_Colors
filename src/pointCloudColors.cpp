
/*-------------------------------------------------------------------*\

  NAME
    pointCloudColors.cpp


  DESCRIPTION
    Colorize Point Cloud based on intensity using OpenCV Colormaps

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/

#include "pointCloudUtils.h"
#include "pointCloudColors.h"

/*-------------------------------------------------------------------*\
  Colorize Point Cloud in false color based on intensity 
\*-------------------------------------------------------------------*/
void colorizeIntensityPointCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudI,   // XYZI Input Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,  // XYZRGB Point Cloud with colorw
    int colorMap,                                       // OpenCV ColorMap to use
    bool bDebugPrint
)

{
    // Check intensity value range
    float minIntensity, maxIntensity;
    getIntensityRange(pointCloudI, minIntensity, maxIntensity, bDebugPrint);

    int nRows = 1, nCols = pointCloudI->points.size();
    if (bDebugPrint) {
        std::cout << "Number of columns: " << nCols << std::endl;
    }
    // Transfer Point Cloud into OpenCV Matrix so that we can apply OpenCV color maps
    cv::Mat grayImg;
    grayImg = cv::Mat::zeros( nRows, nCols, CV_8UC1);
    
    int i = 0;
    for (auto it = pointCloudI->points.begin(); it != pointCloudI->points.end(); it++) {
        // Scale intensitities between 0 and 255
        int val = std::min(255, (int)(255 * (1 - abs((it->intensity - maxIntensity) / maxIntensity))));
        grayImg.data[i] = static_cast<uchar>(val);
        i++;
    }
    if (bDebugPrint) {
        std::cout << "Copied " << i << " points into Image" << std::endl;
    }
    if (nCols != i) {
        std::cerr << "***ERROR: Index Mismatch"  << std::endl;
    }

    // Apply OpenCV ColorMap
    cv::Mat falseColorImg;
    cv::applyColorMap(grayImg, falseColorImg, colorMap);
    
    // Transfer from OpenCV Matrix to XYZRGB Point Cloud
    i = 0;
    for (auto it = pointCloudI->points.begin(); it != pointCloudI->points.end(); it++) 
    {
        pcl::PointXYZRGB point;
        point.x = it->x; 
        point.y = it->y; 
        point.z = it->z; 

        int red = falseColorImg.at<cv::Vec3b>(0,i)[2];
        int green = falseColorImg.at<cv::Vec3b>(0,i)[1];
        int blue = falseColorImg.at<cv::Vec3b>(0,i)[0];
        i++;
        uint32_t rgb = (static_cast<uint32_t>(red) << 16 | static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        pointCloud->points.push_back(point);

    }
    pointCloud->width = (int)pointCloud->points.size();
    pointCloud->height = 1;

}