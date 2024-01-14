#include "preprocess.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include "rotation.h"
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
bool find_file_under_dir(const string& AbsFilePath, vector<string>& FileName) {
    DIR* dir;
    struct dirent* ptr;
    if (!(dir = opendir(AbsFilePath.c_str()))) {
        cout << "current dir isn't exit" << endl;
        return false;
    }
    while ((ptr = readdir(dir)) != 0) {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
            continue;
        }
        FileName.push_back(string(ptr->d_name));
    }
    closedir(dir);
    return true;
}
void preprocess_rawdata(string img_folder, string pcd_folder, string output_pcd_path) {
    //define img pcd pair
    std::vector<std::pair<std::string, std::string>> img_pcd_vec;
    vector<string> img_name;
    if (find_file_under_dir(img_folder + "/left", img_name)) {
        cout << "load image : done" << endl;
    }
    vector<string> pcd_name;
    if (find_file_under_dir(pcd_folder, pcd_name)) {
        cout << "load pcd : done" << endl;
    }
    //find the closest pcd to current image(no matter fisheye or pinhole)
    for (int i = 0; i < img_name.size(); i++) {
        double current_img_timestamp = std::stoi(img_name[i].substr(0, img_name[i].length() - 4));
        //define
        //double min_diff = std::abs(std::stoi(pcd_name[0].substr( 0, pcd_name[0].length() - 4)) - current_img_timestamp);
        bool find = false;
        std::string closest_element = pcd_name[0];
        for (int j = 0; j < pcd_name.size(); j++) {
            double diff =
                std::stoi(pcd_name[j].substr(0, pcd_name[j].length() - 4)) - current_img_timestamp;
            if (diff < 0.1 && diff >= 0) {
                //min_diff = diff;
                closest_element = pcd_name[j];  //find 0<pcd_timestamp-img_timestamp<0.1
                find = true;
            }
        }
        if (find) {
            img_pcd_vec.push_back(std::make_pair(img_name[i], closest_element));
        } else {
            int rm1 = std::remove((img_folder + "/left/" + img_name[i]).c_str());
            if (rm1 != 0) {
                std::cerr << "Failed to delete the file.\n";
            } else {
                std::cout << "The file has been deleted.\n";
            }
            cout << (img_folder + "/left/" + img_name[i]) << endl;
            int rm2 = std::remove((img_folder + "/right/" + img_name[i]).c_str());
            if (rm2 != 0) {
                std::cerr << "Failed to delete the file.\n";
            } else {
                std::cout << "The file has been deleted.\n";
            }
            cout << (img_folder + "/right/" + img_name[i]) << endl;
        }
    }
    //output closet pcd timestamp about image
    for (const auto& p : img_pcd_vec) {
        std::cout << p.first << " : " << p.second << std::endl;
    }
    //read pose and save in a varaible
    /*
        after know pose type ,I will write code
    */
    for (int i = 0; i < img_pcd_vec.size(); i++) {
        pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
        if (pcl::io::loadPCDFile<pcl::PointXYZIR>(pcd_folder + "/" + img_pcd_vec[i].second,
                                                  *input_pcd) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            system("PAUSE");
        }
        //do pointcloud motion compsention
        Matrix3d rvec;  //read from pose file
        rvec << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Vector3d tvec;  //read from pose file
        tvec << 0, 0, 0;
        //define output pcd
        pcl::PointCloud<pcl::PointXYZIR>::Ptr undistort_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
        pcl::PointCloud<pcl::PointXYZIR>::Ptr sync_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
        double current_img_timestamp =
            std::stoi(img_pcd_vec[i].first.substr(0, img_pcd_vec[i].first.length() - 4));
        double current_pcd_timestamp =
            std::stoi(img_pcd_vec[i].second.substr(0, img_pcd_vec[i].second.length() - 4));
        //pointcloud undistort
        double delta_time = 0.1;
        point_undistort(input_pcd, rvec, tvec, current_pcd_timestamp, delta_time, undistort_pcd);
        Matrix3d tc_R_t1;
        tc_R_t1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Vector3d tc_t_t1;
        tc_t_t1 << 0, 0, 0;
        tranform_pcd(undistort_pcd, tc_R_t1, tc_t_t1, sync_pcd);
        //save tranform pointcloud to outputfolder
        pcl::io::savePCDFileASCII(
            output_pcd_path + "/" +
                img_pcd_vec[i].first.substr(0, img_pcd_vec[i].first.length() - 4) + ".pcd",
            *sync_pcd);
    }
}
int main(int argc, char** argv) {
    //define pinhole_img_folder path
    //string pinhole_img_folder="/home/ljj/work/lidar-cam-calibration/data/pinhole/left";
    string root_folder(argv[1]);
    string pinhole_img_folder = root_folder + "/pinhole";
    string fisheye_img_folder = root_folder + "/fisheye";
    string pcd_folder = root_folder + "/pcd";
    std::string pinhole_pcd_folder = root_folder + "/pinhole_pcd";
    std::string fisheye_pcd_folder = root_folder + "/fisheye_pcd";
    //create output two pcd path
    cout << mkdir(pinhole_pcd_folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    cout << mkdir(fisheye_pcd_folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    //deal pinhole image
    preprocess_rawdata(pinhole_img_folder, pcd_folder, pinhole_pcd_folder);
    //deal fisheye image
    preprocess_rawdata(fisheye_img_folder, pcd_folder, fisheye_pcd_folder);
}
