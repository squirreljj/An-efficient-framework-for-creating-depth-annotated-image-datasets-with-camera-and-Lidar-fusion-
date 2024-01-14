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
#include "motion_compensation.h"
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
                     pcl::PointCloud<pcl::PointXYZIR>::Ptr& undistort_pcd) {
    //interpolatetransform
    for (const auto& point : *input_pcd) {
        //output every point info
        /*std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << " intensity: " << point.intensity << " ring: " << 
        static_cast<int>(point.ring) << " timestamp: " << std::fixed << std::setprecision(16)<< point.timestamp << std::endl;*/
        //transform points t1_T_tp
        Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q2(rvec);
        double ratios = (current_pcd_timestamp - point.timestamp) / delta_time;
        Matrix3d t1_R_tp = q2.slerp(ratios, q1).toRotationMatrix();
        Vector3d t1_t_tp = -tran * ratios;
        Vector3d position;
        position[0] = point.x;
        position[1] = point.y;
        position[2] = point.z;
        //tranform every point t1_T_tp
        position = t1_R_tp * position + t1_t_tp;  //get undistort postion
        //put position to undistort_pcd
        pcl::PointXYZIR new_point;
        new_point.x = position[0];
        new_point.y = position[1];
        new_point.z = position[2];
        //new_point.intensity = point.intensity;
        new_point.timestamp = current_pcd_timestamp;
        //new_point.ring = point.ring;
        undistort_pcd->push_back(new_point);
    }
}
void tranform_pcd(pcl::PointCloud<pcl::PointXYZIR>::Ptr& undistort_pcd, Matrix3d tc_R_t1,
                  Vector3d tc_t_t1, pcl::PointCloud<pcl::PointXYZIR>::Ptr& sync_pcd) {
    //interpolatetransform
    //tranforn point from t1->tc
    for (const auto& point : *undistort_pcd) {
        //put position to sync_pcd
        Vector3d position;
        position[0] = point.x;
        position[1] = point.y;
        position[2] = point.z;
        position = tc_R_t1 * position + tc_t_t1;
        pcl::PointXYZIR new_point;
        new_point.x = position[0];
        new_point.y = position[1];
        new_point.z = position[2];
        //new_point.intensity = point.intensity;
        new_point.timestamp = point.timestamp;
        //new_point.ring = point.ring;
        sync_pcd->push_back(new_point);
    }
}
