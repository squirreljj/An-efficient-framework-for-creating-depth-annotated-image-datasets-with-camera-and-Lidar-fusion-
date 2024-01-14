// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <dirent.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <yaml-cpp/yaml.h>
//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMappingClass.h"
#include "lidar.h"

#define pcd_save_en 1
int pcd_index = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_wait_save(new pcl::PointCloud<pcl::PointXYZI>());
int file_count = 0;
int count=0;
LaserMappingClass laserMapping;
lidar::Lidar lidar_param;
std::mutex mutex_lock;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
bool program_start=false;
bool map_finish=false;
ros::Publisher map_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}


void laser_mapping(){
    while(1){
        if(!odometryBuf.empty() && !pointCloudBuf.empty()){
            //read data
            program_start=true;
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
                odometryBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            

            laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);
            // 读取YAML文件
	    std::ifstream fin("/home/ljj/work/lidar-cam-calibration/start.yaml");
	    YAML::Node node = YAML::Load(fin);
	    // 获取字符串
	    std::string pose_map_path = node["pose_map_path"].as<std::string>();
	    // 输出字符串

	    //std::string pose_map_path = "/home/ljj/work/lidar-cam-calibration/preprocess/data/pose_map.txt";
	    //add code by ljj
	    // 打开文件流，以追加的方式写入文件
	    std::ofstream outfile;
	    outfile.open(pose_map_path, std::ios_base::app);

	    outfile <<pointcloud_time << " ";
	    // 将旋转向量写入文件
	     // 从等距变换中提取旋转矩阵
    	    Eigen::Matrix3d R = current_pose.rotation();

	    // 将旋转矩阵转换为四元数
	    Eigen::Quaterniond q(R);

	    // 将四元数转换为旋转向量
	    Eigen::AngleAxisd aa(q);
	    outfile << R(0,0) << " "<< R(0,1) << " "<< R(0,2) << " "<< R(1,0) << " "<< R(1,1) << " "<< R(1,2) << " "<< R(2,0) << " "<< R(2,1) << " "<< R(2,2) << " ";

	    // 将平移向量写入文件
	    Eigen::Vector3d translation_vector = current_pose.translation();
	    outfile  << translation_vector.transpose() << std::endl;



            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*pc_map, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "/map";
            map_pub.publish(PointsMsg); 
            count++;
            std::cout<<count<<std::endl;
	    if(count==file_count){
	 	std::cout<<"yes"<<std::endl;
	    	system("pkill -f roslaunch");
	    }
            // save full pointcloud
            if (pcd_save_en)
            {
                int size = pointcloud_in->points.size();
                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>(size, 1));
	            pcl::transformPointCloud(*pointcloud_in, *transformed_pc, current_pose.cast<float>());
                
                *pcl_wait_save += *transformed_pc;

                // if (pcl_wait_save->size() > 0)
                // {
                //     pcd_index ++;
                //     std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + std::to_string(pcd_index) + std::string(".pcd"));
                //     pcl::PCDWriter pcd_writer;
                //     std::cout << "current scan saved to /PCD/" << all_points_dir << std::endl;
                //     pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
                //     pcl_wait_save->clear();
                // }
            }
        }
        
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    DIR *dir;
    struct dirent *ent;
    std::string my_param;
    ros::param::param<std::string>("my_param", my_param, "/default_value");
    std::ifstream fin(my_param);
    YAML::Node node = YAML::Load(fin);
    std::string pcd_folder = node["pcd_folder"].as<std::string>();
    if ((dir = opendir (pcd_folder.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                file_count++;
            }
        }
        closedir (dir);
        printf("Number of files in %s: %d\n", argv[1], file_count);
    }

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;    
    std::string ROOT_DIR;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/map_save_ROOT", ROOT_DIR);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserMapping.init(map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread laser_mapping_process{laser_mapping};

    ros::spin();

    // exit the progroam and save the full map
    if (pcl_wait_save->size() > 0)
    {
        // pcd_index ++;
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "/PCD/map_") + std::string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/:" << all_points_dir << std::endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
    }

    return 0;
}
