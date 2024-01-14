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
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
namespace pcl {
struct PointXYZIR {
    PCL_ADD_POINT4D;                 // 继承父类的x、y、z三个字段
    float intensity;                 // 强度值
    uint16_t ring;                   // 激光雷达线号
    double timestamp;                // 时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 对齐内存
} EIGEN_ALIGN16;
};  // namespace pcl
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))
