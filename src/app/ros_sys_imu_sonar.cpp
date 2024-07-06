#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <deque>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <uuv_rexrov_sonar/wrapper/ieskf_frontend_noetic_wrapper.h>
#include "pcl/common/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "uuv_rexrov_sonar/math/math.hpp"
#include "uuv_rexrov_sonar/type/pointcloud.h"

std::deque<sensor_msgs::PointCloud2> sonar_deque;
std::deque<nav_msgs::Odometry> gt_deque;

const int timestamp_tolerance_ns = 10000000;

pcl::PointCloud<pcl::PointXYZ> global_cloud_;

void doSonarMsg(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sonar_deque.push_back(*msg);
}

void doGtMsg(const nav_msgs::Odometry::ConstPtr &msg)
{
    gt_deque.push_back(*msg);
}


bool sysSoanrGt(sensor_msgs::PointCloud2 &sonar, nav_msgs::Odometry &gt)
{
    if (sonar_deque.empty() || gt_deque.empty())
    {
        return false;
    }

    while (!sonar_deque.empty() && !gt_deque.empty())
    {
        const auto &sonar_msg = sonar_deque.front();
        const auto &gt_msg = gt_deque.front();


        int sec_diff = sonar_msg.header.stamp.sec - gt_msg.header.stamp.sec;
        int nsec_diff = sonar_msg.header.stamp.nsec - gt_msg.header.stamp.nsec;


        int timestamp_diff = abs(sec_diff) * 1000000000 + abs(nsec_diff);

        if (timestamp_diff <= timestamp_tolerance_ns)
        {

            sonar = sonar_msg;
            gt = gt_msg;

            sonar_deque.pop_front();
            gt_deque.pop_front();
            std::cout<<"找到对应imu和sonar"<<std::endl;
            return true;
        }
        else if (sonar_msg.header.stamp < gt_msg.header.stamp)
        {

            sonar_deque.pop_front();
        }
        else
        {

            gt_deque.pop_front();
        }
    }


    return false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gt_pose");
    ros::NodeHandle nh;

    ros::Subscriber sonar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rov/lidar", 100, doSonarMsg);
    ros::Subscriber gt_sub = nh.subscribe<nav_msgs::Odometry>("/rexrov1/pose_gt", 100, doGtMsg);

    ros::Publisher gt_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/rov/gt_lidar", 100);

    ros::Rate rate(500);

    sensor_msgs::PointCloud2 sonar_lidar;
    nav_msgs::Odometry gt_pose;

    while (ros::ok())
    {
        if (sysSoanrGt(sonar_lidar, gt_pose))
        {
            Eigen::Quaterniond gt_rotation(gt_pose.pose.pose.orientation.x,
                                           gt_pose.pose.pose.orientation.y,
                                           gt_pose.pose.pose.orientation.z,
                                           gt_pose.pose.pose.orientation.w);
            Eigen::Vector3d gt_position(gt_pose.pose.pose.position.x,
                                        gt_pose.pose.pose.position.y,
                                        gt_pose.pose.pose.position.z);



            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(sonar_lidar,cloud);
            pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(gt_rotation, gt_position).cast<float>());


            global_cloud_ += cloud;
            sensor_msgs::PointCloud2 msg;
            std::cout << "global cloud size: " << global_cloud_.size() << std::endl;

            pcl::toROSMsg(global_cloud_, msg);
            msg.header.frame_id = "world";
            gt_lidar_pub.publish(msg);
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
