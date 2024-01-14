#include "read_pcd.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <iomanip>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "please type input pcd path!" << std::endl;
        return -1;
    }
    // read pointcloud
    pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>);
    if (pcl::io::loadPCDFile<pcl::PointXYZIR>(argv[1], *cloud) == -1) {
        std::cerr << "cant read pcd file " << argv[1] << "!" << std::endl;
        return -1;
    }
    // output pcd infos in terminal
    std::cout << "pointcloud size is : " << cloud->size() << std::endl;
    int count = 0;
    double max, min;
    for (const auto& point : *cloud) {
        if (count == 0) {
            max = point.timestamp;
            min = point.timestamp;
            count++;
        }
        if (point.timestamp > max)
            max = point.timestamp;
        if (point.timestamp < min)
            min = point.timestamp;
        std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z
                  << " intensity: " << point.intensity << " ring: " << static_cast<int>(point.ring)
                  << " timestamp: " << std::fixed << std::setprecision(16) << point.timestamp
                  << std::endl;
    }
    cout << max << " " << min << endl;
    return 0;
}
