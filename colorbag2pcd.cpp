#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB> livox_cloud_single_frame,livox_cloud; 
string input_bag_path, output_path;
int threshold_lidar;

void loadAndSavePointcloud();

void loadAndSavePointcloud() {
    string path = input_bag_path;
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("sensor_msgs/PointCloud2")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    for (const rosbag::MessageInstance& m : view) {
        pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2 >()), livox_cloud_single_frame);
        if (0 == cloudCount){
            livox_cloud = livox_cloud_single_frame;
        }
        else{
            livox_cloud += livox_cloud_single_frame;
        }
        pcl::io::savePCDFileASCII(output_path, livox_cloud);
        // std::cout<< livox_cloud.point_num<<std::endl; // mid40：一帧10000个点；1秒10帧率
        ++cloudCount;
        if (threshold_lidar == 0){
            continue;
        }
        if (cloudCount >= threshold_lidar) {
            break;
        }
    }
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else {
        cout << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "colorbag2pcd");
    getParameters();
    loadAndSavePointcloud();
    ROS_INFO("Finish!");
    return 0;
}

