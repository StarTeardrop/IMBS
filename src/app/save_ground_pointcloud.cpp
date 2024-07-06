#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher pub;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    pcl::PointCloud<pcl::PointXYZ> current_cloud;
    pcl::fromROSMsg(*cloud_msg, current_cloud);
    

    if (current_cloud.empty()) {
        ROS_WARN("Received empty point cloud, skipping...");
        return;
    }
    

    *accumulated_cloud += current_cloud;
    

    ROS_INFO("Accumulated cloud size: %lu points", accumulated_cloud->size());
}


void saveAndPublishPointCloudTimerCallback(const ros::TimerEvent& event) {

    if (accumulated_cloud->empty()) {
        ROS_WARN("No accumulated point cloud data to save and publish.");
        return;
    }
    

    std::string file_path = "/home/robot/pointcloud_accumulated.pcd";
    

    pcl::io::savePCDFileASCII(file_path, *accumulated_cloud);
    
    ROS_INFO("Saved accumulated point cloud to file: %s", file_path.c_str());
    

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*accumulated_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    
    pub.publish(cloud_msg);
    
    ROS_INFO("Published accumulated point cloud with %lu points", accumulated_cloud->size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_accumulator");
    ros::NodeHandle nh;
    

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/rov/lidar", 1, cloudCallback);
    

    pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_pointcloud", 1);
    

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), saveAndPublishPointCloudTimerCallback);
    

    accumulated_cloud->clear();
    

    ros::spin();
    
    return 0;
}
