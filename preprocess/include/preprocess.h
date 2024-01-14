#ifndef PREPROCESS_H
#define PREPROCESS_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <cstdio>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <unistd.h>
#include <vector>
#include "motion_compensation.h"
void Stringsplit(const string& str, const string& splits, vector<string>& res);
bool find_file_under_dir(const string& AbsFilePath, vector<string>& FileName);
void preprocess_rawdata(string img_folder, string pcd_folder, string output_pcd_path, string delta_pose_path);

#endif 
