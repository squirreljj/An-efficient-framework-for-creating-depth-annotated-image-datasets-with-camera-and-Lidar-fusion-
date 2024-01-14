#ifndef MOTION_COMPENSATION_H
#define MOTION_COMPENSATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include "read_pcd.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
void point_undistort(pcl::PointCloud<pcl::PointXYZIR>::Ptr& input_pcd, Matrix3d rvec, Vector3d tran,
                     double current_pcd_timestamp, double delta_time,
                     pcl::PointCloud<pcl::PointXYZIR>::Ptr& undistort_pcd);
 
void tranform_pcd(pcl::PointCloud<pcl::PointXYZIR>::Ptr& undistort_pcd, Matrix3d tc_R_t1,
                  Vector3d tc_t_t1, pcl::PointCloud<pcl::PointXYZIR>::Ptr& sync_pcd);
#endif 
