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
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace cv;
using namespace pcl;
void Stringsplit(const string& str, const string& splits, vector<string>& res)
{
    if (str == "")		
        return;
    string strs = str + splits;
    size_t pos = strs.find(splits);
    int step = splits.size();
    while (pos != strs.npos){
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        strs = strs.substr(pos + step, strs.size());
        pos = strs.find(splits);
    }
}
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
void preprocess_rawdata(string img_folder, string pcd_folder, string output_pcd_path,string delta_pose_path) {
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
        double current_img_timestamp = std::stod(img_name[i].substr(0, img_name[i].length() - 4));
        //define
        //double min_diff = std::abs(std::stod(pcd_name[0].substr( 0, pcd_name[0].length() - 4)) - current_img_timestamp);
        bool find = false;
        std::string closest_element = pcd_name[0];
        for (int j = 0; j < pcd_name.size(); j++) {
            double diff =
                std::stod(pcd_name[j].substr(0, pcd_name[j].length() - 4)) - current_img_timestamp;
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
    cout<<img_pcd_vec.size()<<endl;
    /*for (const auto& p : img_pcd_vec) {
        std::cout << p.first << " : " << p.second << std::endl;
    }*/
    //read pose and save in a varaible
    /*
        after know pose type ,I will write code
    */
    //read delta pose from pose_scan.txt
    
    ifstream file(delta_pose_path);
    string str;
    vector<string> data;
    while(getline(file,str)){		
        vector<string> list_str;
        Stringsplit(str," ",list_str);
        data.insert(data.end(),list_str.begin(),list_str.end());
    }
    
    auto isEmptyString = [](const string& str) { return str.empty(); };
    auto new_end = std::remove_if(data.begin(), data.end(), isEmptyString);
    data.erase(new_end, data.end());//remove null rows
    
    for (int i = 0; i < img_pcd_vec.size(); i++) {
        pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
        if (pcl::io::loadPCDFile<pcl::PointXYZIR>(pcd_folder + "/" + img_pcd_vec[i].second,
                                                  *input_pcd) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            system("PAUSE");
        }
  
        Matrix3d rvec;  //read from pose file
        Vector3d tvec;  //read from pose file
        //find current pcd correspond timestamp from delta pose txt 
        bool flag=false;
        for(int j=0;j<data.size();j++){
            //cout<<data[j]<<" "<<img_pcd_vec[i].second.substr(0,img_pcd_vec[i].second.length() - 4)<<endl;
            if( data[j].substr(0,data[j].size()-4) == img_pcd_vec[i].second.substr(0,img_pcd_vec[i].second.length() - 8)){
		        //cout<<"find pose"<<endl;
                flag=true;
                rvec << stod(data[j+1]),stod(data[j+2]),stod(data[j+3]),stod(data[j+4]),stod(data[j+5]),stod(data[j+6]),stod(data[j+7]),stod(data[j+8]),stod(data[j+9]);
                tvec << stod(data[j+10]),stod(data[j+11]),stod(data[j+12]);
            }
        }
        //cout<<flag<<endl;
        if(flag==false){
            cout<<img_pcd_vec[i].second.substr(0,img_pcd_vec[i].second.length() - 4)<<endl;
            rvec<<1,0,0,0,1,0,0,0,1;
            tvec<<0,0,0;
        }
        //do pointcloud motion compsention
        //define output pcd
        pcl::PointCloud<pcl::PointXYZIR>::Ptr undistort_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
        pcl::PointCloud<pcl::PointXYZIR>::Ptr sync_pcd(new pcl::PointCloud<pcl::PointXYZIR>);
	
        double current_img_timestamp =
            std::stod(img_pcd_vec[i].first.substr(0, img_pcd_vec[i].first.length() - 4));
        //cout<<"current_img_timestamp:"<<current_img_timestamp<<endl;
        double current_pcd_timestamp =
            std::stod(img_pcd_vec[i].second.substr(0, img_pcd_vec[i].second.length() - 4));
        //cout<<"current_pcd_timestamp"<<current_pcd_timestamp<<endl;
        //pointcloud undistort
        double delta_time = 0.1;
        point_undistort(input_pcd, rvec, tvec, current_pcd_timestamp, delta_time, undistort_pcd);
        //get tc_T_t1
        Matrix3d tc_R_t1;
        Vector3d tc_t_t1;
        Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q2(rvec);
        tc_R_t1=q1.slerp((current_pcd_timestamp-current_img_timestamp)/delta_time, q2).toRotationMatrix();
        tc_t_t1=tvec*(current_pcd_timestamp-current_img_timestamp)/delta_time;
        tranform_pcd(undistort_pcd, tc_R_t1, tc_t_t1, sync_pcd);
        //save tranform pointcloud to outputfolder
        pcl::io::savePCDFileASCII(
            output_pcd_path + "/" +
                img_pcd_vec[i].first.substr(0, img_pcd_vec[i].first.length() - 4) + ".pcd",
            *undistort_pcd);
    }
}
int main(int argc, char** argv) {
    string yaml_path=argv[1];
    std::ifstream fin(yaml_path);
    YAML::Node node = YAML::Load(fin);
    std::string root_folder = node["root_folder"].as<std::string>();
    std::string delta_pose_path = node["delta_pose_path"].as<std::string>();
    //string root_folder;
    //string delta_pose_path(argv[2]);
    string pinhole_img_folder = root_folder + "/pinhole";
    string fisheye_img_folder = root_folder + "/fisheye";
    string pcd_folder = root_folder + "/pcd";
    std::string pinhole_pcd_folder = root_folder + "/pinhole_pcd";
    std::string fisheye_pcd_folder = root_folder + "/fisheye_pcd";
    //create output two pcd path
    cout << mkdir(pinhole_pcd_folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    cout << mkdir(fisheye_pcd_folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    //deal pinhole image
    preprocess_rawdata(pinhole_img_folder, pcd_folder, pinhole_pcd_folder,delta_pose_path);
    //deal fisheye image
    preprocess_rawdata(fisheye_img_folder, pcd_folder, fisheye_pcd_folder,delta_pose_path);
}
