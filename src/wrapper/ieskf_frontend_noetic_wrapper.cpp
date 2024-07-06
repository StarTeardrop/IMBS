#include "uuv_rexrov_sonar/wrapper/ieskf_frontend_noetic_wrapper.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "uuv_rexrov_sonar/math/math.hpp"
#include <Eigen/Geometry>

namespace ROSNoetic
{
    double depth_m;
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh)
    {
        front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>();


        cloud_subscriber = nh.subscribe("/rov/lidar", 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe("/rov/origin_imu", 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        depth_subscriber = nh.subscribe("/rov/depth", 100, &IESKFFrontEndWrapper::depthMsgCallBack, this);
        gt_subscriber = nh.subscribe("/rexrov1/pose_gt", 100, &IESKFFrontEndWrapper::gtMsgCallBack, this);

        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rov/curr_cloud", 100);
        path_pub = nh.advertise<nav_msgs::Path>("/rov/path", 100);

        gt_path_pub = nh.advertise<nav_msgs::Path>("/rov/gt_path", 100);


        cloud_pose_pub = nh.advertise<uuv_rexrov_sonar::CloudWithPose>("/rov/cloud_with_pose", 100);

        run();
    }

    IESKFFrontEndWrapper::~IESKFFrontEndWrapper()
    {
    }

    void IESKFFrontEndWrapper::gtMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
    {
        gt_pose = *msg;
    }

    void IESKFFrontEndWrapper::depthMsgCallBack(const sensor_msgs::FluidPressure::ConstPtr &msg)
    {
        float fluidpressure = msg->fluid_pressure;

        depth_m = fluidpressure / (1000 * IESKFSlam::GRAVITY);
    }

    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg)
    {
        IESKFSlam::PointCloud cloud;

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        cloud.cloud_ptr->clear();
        for (auto &&point : pcl_cloud)
        {
            IESKFSlam::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            cloud.cloud_ptr->push_back(p);
        }
        cloud.time_stamp.fromNsec(msg->header.stamp.toNSec());
        front_end_ptr->addPointCloud(cloud);
    }

    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg)
    {
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }

    void IESKFFrontEndWrapper::run()
    {
        ros::Rate rate(500);
        while (ros::ok())
        {
            rate.sleep();
            ros::spinOnce();
            if (front_end_ptr->track())
            {
                publishMsg();
            }
        }
    }

    void IESKFFrontEndWrapper::publishMsg()
    {

        static nav_msgs::Path gt_path;
        gt_path.header.frame_id = "world";
        geometry_msgs::PoseStamped gt_psd;
        gt_psd.pose.position.x = gt_pose.pose.pose.position.x;
        gt_psd.pose.position.y = gt_pose.pose.pose.position.y;
        gt_psd.pose.position.z = gt_pose.pose.pose.position.z;
        gt_psd.pose.position.z = depth_m;
        // ROS_INFO("gt_pose: x: %f, y: %f, z: %f", gt_pose.pose.pose.position.x, gt_pose.pose.pose.position.y, gt_pose.pose.pose.position.z);
        gt_path.poses.push_back(gt_psd);
        gt_path_pub.publish(gt_path);


        // std::ofstream gt_file;
        // gt_file.open(gt_file_path, std::ios::out | std::ios::app);
        // if (gt_file.is_open())
        // {
        //
        //     gt_file << gt_pose.pose.pose.position.x << " " << gt_pose.pose.pose.position.y << " " << gt_pose.pose.pose.position.z << " "
        //             << gt_pose.pose.pose.orientation.x << " " << gt_pose.pose.pose.orientation.y << " " << gt_pose.pose.pose.orientation.z << " "
        //             << gt_pose.pose.pose.orientation.w << std::endl;
        //     gt_file.close();
        //     ROS_INFO("Saved gt_path data to file.");
        // }
        // else
        // {
        //     ROS_ERROR("Unable to open file.");
        // }

        static nav_msgs::Path path;
        auto X = front_end_ptr->readState();
        path.header.frame_id = "world";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.y = X.position.x();
        psd.pose.position.x = -X.position.y();
        psd.pose.position.z = X.position.z();
        psd.pose.position.z = depth_m;
        // std::cout << "depth: " << depth_m << std::endl;
        ROS_INFO("x: %f, y: %f, z: %f", psd.pose.position.x, psd.pose.position.y, psd.pose.position.z);

        path.poses.push_back(psd);
        path_pub.publish(path);


//        std::ofstream pred_file;
//        pred_file.open(pred_file_path1, std::ios::out | std::ios::app);
//        if (pred_file.is_open())
//        {
//            pred_file << psd.pose.position.x << " " << psd.pose.position.y << " " << psd.pose.position.z << " "
//                      << X.rotation.x() << " " << X.rotation.y() << " " << X.rotation.z() << " "
//                      << X.rotation.w() << std::endl;
//            pred_file.close();
//            ROS_INFO("Saved pred_path data to file.");
//        }
//        else
//        {
//            ROS_ERROR("Unable to open file.");
//        }

        IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();

        pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());

        std::cout << "Quaternion (w, x, y, z): " << X.rotation.w() << ", "
                  << X.rotation.x() << ", " << X.rotation.y() << ", " << X.rotation.z() << std::endl;

        Eigen::Vector3d euler_angles = X.rotation.toRotationMatrix().eulerAngles(2, 1, 0);

        euler_angles = euler_angles * 180.0 / M_PI;

        std::cout << "Euler angles (yaw, pitch, roll) in degrees: " << euler_angles.transpose() << std::endl;

        std::cout << "Position (x, y, z): " << X.position.transpose() << std::endl;


        global_cloud_ += cloud;

        pcl::VoxelGrid<IESKFSlam::Point> voxel_filter;
        voxel_filter.setInputCloud(global_cloud_.makeShared());
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        pcl::PointCloud<IESKFSlam::Point> filtered_cloud;
        voxel_filter.filter(filtered_cloud);
        global_cloud_ = filtered_cloud;

        sensor_msgs::PointCloud2 msg;
        std::cout << "global cloud size: " << global_cloud_.size() << std::endl;
        // pcl::toROSMsg(cloud, msg);
        pcl::toROSMsg(global_cloud_, msg);
        msg.header.frame_id = "world";
        curr_cloud_pub.publish(msg);
        std::cout << "mls cloud size: " << msg.data.size() << std::endl;


        uuv_rexrov_sonar::CloudWithPose cloud_with_pose_msg;
        cloud = front_end_ptr->readCurrentFullPointCloud();
        pcl::toROSMsg(cloud, cloud_with_pose_msg.point_cloud);
        cloud_with_pose_msg.pose.position = psd.pose.position;
        cloud_with_pose_msg.pose.orientation.x = X.rotation.x();
        cloud_with_pose_msg.pose.orientation.y = X.rotation.y();
        cloud_with_pose_msg.pose.orientation.z = X.rotation.z();
        cloud_with_pose_msg.pose.orientation.w = X.rotation.w();
        cloud_pose_pub.publish(cloud_with_pose_msg);
    }

}
