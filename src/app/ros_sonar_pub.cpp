#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <thread>
#include <pcl/common/centroid.h>
#include "sensor_msgs/FluidPressure.h"
#include "uuv_rexrov_sonar/type/base_type.h"


sensor_msgs::Image image;


ros::Publisher rov_lidar_pub;


float sonar_img_width = 512.0f;
float sonar_img_height = 399.0f;
float sonar_x_range = 10.0f;
float sonar_y_range = 12.83f;


double depth;

void Sub_Rov_Sonar(const sensor_msgs::Image::ConstPtr &msg_image)
{

    image = *msg_image;

    image.header.stamp = ros::Time::now();



    cv_bridge::CvImageConstPtr cv_ptr1;
    try
    {

        cv_ptr1 = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat grayImage1;
    cv::cvtColor(cv_ptr1->image, grayImage1, cv::COLOR_BGR2GRAY);


    int thresholdValue = 200;
    cv::Mat thresholdedImage1;
    cv::threshold(grayImage1, thresholdedImage1, thresholdValue, 255, cv::THRESH_BINARY);

    std::deque<cv::Point> points;
    // Iterate through pixels and save keypoints
    for (int y = 0; y < thresholdedImage1.rows; ++y)
    {
        for (int x = 0; x < thresholdedImage1.cols; ++x)
        {
            if (thresholdedImage1.at<uchar>(y, x) > 0)
            {
                points.push_back(cv::Point(x, y));
            }
        }
    }

    // Convert 2D points to PointCloud<pcl::PointXYZ> format
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    temp_cloud->width = points.size();
    temp_cloud->height = 1;
    temp_cloud->is_dense = true;
    temp_cloud->points.resize(temp_cloud->width * temp_cloud->height);

    // Fill in the PointCloud data
    for (size_t i = 0; i < points.size(); ++i)
    {
        float x_real = sonar_x_range - (float)(points[i].y / sonar_img_height) * sonar_x_range;
        float y_real = (float)(points[i].x / sonar_img_width) * sonar_y_range - (float)(sonar_y_range / 2);

        temp_cloud->points[i].x = x_real;
        temp_cloud->points[i].y = y_real;
        temp_cloud->points[i].z = 0;

    }

    // Define the transformation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Define translation (0, 0, 0) - No translation needed in this case
    transform.translation() << 0, 0, 0.0;
    // Define rotation
    transform.rotate(Eigen::AngleAxisf(3.1416, Eigen::Vector3f::UnitX())); // Rotate around X axis
    transform.rotate(Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()));    // Rotate around Y axis
    transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));      // Rotate around Z axis

    // Apply the transformation to the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*temp_cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*transformed_cloud, cloud_msg);

    cloud_msg.header.frame_id = "rexrov1/base_footprint";
    cloud_msg.header.stamp = image.header.stamp;

    rov_lidar_pub.publish(cloud_msg);
}

void Sub_Rov_Depth(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    float fluidpressure = msg->fluid_pressure;

    depth = fluidpressure / (1000 * IESKFSlam::GRAVITY);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iskfslam_sonar_node");
    ros::NodeHandle nh;


    ros::Subscriber rov_sonar_sub = nh.subscribe<sensor_msgs::Image>("/rexrov1/blueview_p900/sonar_image_new", 100, Sub_Rov_Sonar);


    ros::Subscriber rov_depth_sub = nh.subscribe<sensor_msgs::FluidPressure>("/rov/depth", 100, Sub_Rov_Depth);


    ros::Publisher rov_sonar_pub = nh.advertise<sensor_msgs::Image>("/rov/sonar", 100);


    rov_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/rov/lidar", 100);


    ros::Rate r(5);


    while (ros::ok())
    {

        rov_sonar_pub.publish(image);


        r.sleep();


        ros::spinOnce();
    }

    return 0;
}
