// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>
//ros lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//local lib
#include "lidar.h"
#include "laserProcessingClass.h"
using namespace std;
using namespace pcl;
LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
//std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<pcl::PointCloud<pcl::PointXYZI>> pointCloudBuf;
std::queue<double> pcd_name;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>);

lidar::Lidar lidar_param;

std::string lidar_type;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;
//modify:directly read pcd
size_t convertMyLidarToPCLPointCloud(const sensor_msgs::PointCloud2ConstPtr &pcd_msg, pcl::PointCloud<pcl::PointXYZI> &pcd)
{
  if (pcd_msg)
  {
    size_t pt_size = pcd_msg->width * pcd_msg->height;

    // output.points.resize (pcd_msg->width * pcd_msg->height);
    // output.channels.resize (pcd_msg->fields.size () - 3);
    // Get the x/y/z field offsets
    int x_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "x");
    int y_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "y");
    int z_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "z");
    int inten_idx = sensor_msgs::getPointCloud2FieldIndex(*pcd_msg, "intensity");
    
    int x_offset = pcd_msg->fields[x_idx].offset;
    int y_offset = pcd_msg->fields[y_idx].offset;
    int z_offset = pcd_msg->fields[z_idx].offset;
    int inten_offset = pcd_msg->fields[inten_idx].offset;

    uint8_t x_datatype = pcd_msg->fields[x_idx].datatype;
    uint8_t y_datatype = pcd_msg->fields[y_idx].datatype;
    uint8_t z_datatype = pcd_msg->fields[z_idx].datatype;
    uint8_t inten_datatype = pcd_msg->fields[inten_idx].datatype;

    // Copy the data points
    size_t validCount = 0;
    for (size_t cp = 0; cp < pt_size; ++cp)
    {
      // Copy x/y/z/intensity
      pcl::PointXYZI pt;
      pt.x = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + x_offset], x_datatype);
      pt.y = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + y_offset], y_datatype);
      pt.z = sensor_msgs::readPointCloud2BufferValue<float>(&pcd_msg->data[cp * pcd_msg->point_step + z_offset], z_datatype);

      if (inten_idx >= 0)
        pt.intensity = sensor_msgs::readPointCloud2BufferValue<uint8_t>(&pcd_msg->data[cp * pcd_msg->point_step + inten_offset], inten_datatype);

      pcd.push_back(pt);
      validCount++;
    }
    return validCount;
  }
  return 0;
}

void velodyneHandler(const pcl::PointCloud<pcl::PointXYZI> &laserCloud)
{
    mutex_lock.lock();
    //pointCloudBuf.push(laserCloudMsg);
    pointCloudBuf.push(laserCloud);
    mutex_lock.unlock();
}

double total_time=0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pointcloud_in = pointCloudBuf.front().makeShared();
            /*if(lidar_type == "rsbpearl")
                convertMyLidarToPCLPointCloud(pointCloudBuf.front(), *pointcloud_in);
            else
                pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            */
            //modify by ljj
            int32_t seconds = floor(pcd_name.front()); // 取整数部分作为秒数
            uint32_t nanoseconds = (pcd_name.front() - seconds) * 1e9; // 将小数部分乘以1e9，作为纳秒数
            ros::Time pointcloud_time(seconds, nanoseconds);
            //cout<<pcd_name.front()<<endl;
            //cout<<pointcloud_time<<endl;
            pointCloudBuf.pop();
            pcd_name.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            if(lidar_type == "rsbpearl"){
                laserProcessing.rsbp_featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            }else
                laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "/base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            // sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            // pcl::toROSMsg(*pointcloud_in, laserCloudFilteredMsg);
            // laserCloudFilteredMsg.header.stamp = pointcloud_time;
            // laserCloudFilteredMsg.header.frame_id = "/base_link";
            // pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "/base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "/base_link";
            pubSurfPoints.publish(surfPointsMsg);

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

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("lidar_type",lidar_type);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);
    // 读取YAML文件
    std::string my_param;
    ros::param::param<std::string>("my_param", my_param, "/default_value");

    std::ifstream fin(my_param);
    if (!fin.is_open()) {
        std::cerr << "Failed to open the file.\n";
        return 1;
    }

    YAML::Node node = YAML::Load(fin);
    // 获取字符串
    std::string pcd_folder = node["pcd_folder"].as<std::string>();
    // 输出字符串
    std::cout << pcd_folder << std::endl;
    //std::string pcd_folder = "/home/ljj/work/lidar-cam-calibration/preprocess/data/pcd/";
    // 获取文件夹下所有文件名
    std::vector<std::string> filenames;
    for (const auto& entry : boost::filesystem::directory_iterator(pcd_folder)){
        if (boost::filesystem::is_regular_file(entry.path())){
            filenames.push_back(entry.path().filename().string());
        }
    }
    
        // 按照数字排序
    std::sort(filenames.begin(), filenames.end(), [](const std::string& a, const std::string& b) {
        // 将字符串按照小数点进行分割
        std::vector<std::string> parts_a, parts_b;
        boost::split(parts_a, a, boost::is_any_of("."));
        boost::split(parts_b, b, boost::is_any_of("."));

        // 分别解析小数点前后的数字，并比较它们的大小关系
        if (std::stod(parts_a[0]) != std::stod(parts_b[0])) {
            return std::stod(parts_a[0]) < std::stod(parts_b[0]);
        }
        else if (parts_a.size() > 1 && parts_b.size() > 1) {
            // 对小数点后面的数字进行比较
            return parts_a[1] < parts_b[1];
        }
        else {
            return a < b;
        }
    });
    for(int i=0;i<filenames.size();i++){
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_folder+filenames[i],*laserCloud) == -1) {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
            system("PAUSE");
        }
        cout<<filenames[i]<<endl;
        pointCloudBuf.push(*laserCloud);
        pcd_name.push(stod(filenames[i].substr(0, filenames[i].length() - 4)));
    }
    
    //ros::Subscriber subLaserCloud = nh.subscribe<pcl::PointXYZI>("/velodyne_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1000);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 10000);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 10000); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}

