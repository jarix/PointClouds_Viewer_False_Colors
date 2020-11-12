/*-------------------------------------------------------------------*\

  NAME
    pointCloudViewer.cpp


  DESCRIPTION
    Load a PCD formatted XYZI point cloud file that has been provided 
    as a command line parameter and display it in PCLVisualizer Window.
    
    Colorize based on intensity using OpenCV colormaps.

    Switch between the colormaps by pressing the 'n' key
    
    PCD File format:
    http://pointclouds.org/documentation/tutorials/pcd_file_format.html

  AUTHOR
    Jari Honkanen

\*-------------------------------------------------------------------*/


#include <iostream>
#include <fstream>
#include <numeric>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "pointCloudUtils.h"
#include "pointCloudColors.h"
#include "pointCloudIO.h"
// Help linker
#include "pointCloudIO.cpp"


static bool bUpdateColor = false;

// OpenCV ColorMaps
// enum
// {
//     COLORMAP_AUTUMN = 0,
//     COLORMAP_BONE = 1,
//     COLORMAP_JET = 2,
//     COLORMAP_WINTER = 3,
//     COLORMAP_RAINBOW = 4,
//     COLORMAP_OCEAN = 5,
//     COLORMAP_SUMMER = 6,
//     COLORMAP_SPRING = 7,
//     COLORMAP_COOL = 8,
//     COLORMAP_HSV = 9,
//     COLORMAP_PINK = 10,
//     COLORMAP_HOT = 11
// }
std::vector<std::string> colorMapNames {"Autumn", "Bone", "Jet", "Winter", "Rainbow", "Ocean", "Summer", "Spring", "Cool", "HSV", "Pink", "Hot"};
static int colorMap = cv::COLORMAP_RAINBOW;
const int maxColorMap = cv::COLORMAP_HOT;


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
/*-------------------------------------------------------------------*\
  Keyboard Event Handler
\*-------------------------------------------------------------------*/
{
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
    if (event.getKeySym() == "n" && event.keyDown()) {
        std::cout << "n pressed" << std::endl;
        bUpdateColor = true;
    }
}


int main(int argc, char *argv[])
/*-------------------------------------------------------------------*\
\*-------------------------------------------------------------------*/
{
    if (argc < 2) {
        std::cout << argv[0] << ": View point cloud with false colored intensity field" << std::endl;
        std::cout << "Usage: " << argv[0] << " input_PCD_filename" << std::endl;
        
        return 1;
    }

    // Create Points clouds Point Cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudI (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load Point Cloud
    pointCloudI = loadPcd<pcl::PointXYZI>(argv[1]);
    if (!pointCloudI) {
        std::cerr << "*** Error: Point Cloud file'" << argv[1] << "' not found." << std::endl;
        return 1;
    }

    // Some some point clouds properties
    pcl::PointXYZI minPoint, maxPoint;
    getPointCloudRange(pointCloudI, minPoint, maxPoint, true);

    pcl::PointXYZ origin;
    getSensorOrigin(pointCloudI, origin, true);
    
    // Colorize the point cloud
    colorizeIntensityPointCloud(pointCloudI, pointCloud, colorMap, true);

    std::string pointCloudName = "Point Cloud";
    std::string visualizerName = "Cyclone Intensity Viewer";

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer(visualizerName));
    
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    // Visualize the color Point Cloud
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, pointCloudName);
    viewer->addText(colorMapNames[colorMap],20, 20, 20, 1, 1, 1, "id");

  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, pointCloudName);
    viewer->initCameraParameters();
    viewer->resetCamera();

    pcl::visualization::Camera cam; 
    viewer->getCameraParameters(cam);
    std::cout << "Camera Parameters:" << std::endl;
    std::cout << "Cam Position: (" << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")" << std::endl; 
    std::cout << "Cam View: (" << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")" << std::endl; 
    std::cout << "Cam Focal: (" << cam.focal[0] << ", " << cam.focal[1] << ", " << cam.focal[2] << ")" << std::endl;
    cam.view[0] = -1;
    cam.view[1] = -1;
    cam.view[2] = 1;
    viewer->setCameraParameters(cam);

    Eigen::Quaternionf orientation = pointCloud->sensor_orientation_;
    std::cout << "Debug: " << "orientation.w() = " << orientation.w() << std::endl; //Print out the scalar
    std::cout << "Debug: " << "orientation.vec() = " << orientation.vec() << std::endl; //Print out the orientation vector
    
    // Main Loop
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (bUpdateColor) {
            
            // Rotate to the next color map
            colorMap++;
            colorMap = (colorMap > maxColorMap ? 0 : colorMap);
            colorizeIntensityPointCloud(pointCloudI, pointCloud, colorMap, true);

            viewer->removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
            viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, pointCloudName);
            viewer->updateText(colorMapNames[colorMap],20, 20, 20, 1, 1, 1, "id");
            
            bUpdateColor = false;
        }
    }

    return 0;
}


