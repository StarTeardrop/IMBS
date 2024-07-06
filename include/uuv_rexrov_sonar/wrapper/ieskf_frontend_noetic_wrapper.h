#pragma once
#include "uuv_rexrov_sonar/modules/frontend/frontend.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/FluidPressure.h"
#include <pcl/surface/mls.h> 
#include "uuv_rexrov_sonar/CloudWithPose.h"

namespace ROSNoetic
{
    class IESKFFrontEndWrapper
    {
    private:
        IESKFSlam::FrontEnd::Ptr front_end_ptr;
        ros::Subscriber cloud_subscriber;
        ros::Subscriber imu_subscriber;
        ros::Subscriber depth_subscriber;

        ros::Subscriber gt_subscriber;
        nav_msgs::Odometry gt_pose;

        ros::Publisher curr_cloud_pub;
        ros::Publisher path_pub;
        ros::Publisher gt_path_pub;


        ros::Publisher cloud_pose_pub;

        pcl::PointCloud<IESKFSlam::Point> global_cloud_;
        // now status
        IESKFSlam::PCLPointCloud curr_cloud;
        Eigen::Quaterniond curr_q;
        Eigen::Vector3d curr_t;

        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
        void depthMsgCallBack(const sensor_msgs::FluidPressure::ConstPtr &msg);
        void gtMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

        void run();
        void publishMsg();

    public:
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
        ~IESKFFrontEndWrapper();


//        std::string gt_file_path = "/home/robot/pointcloud_sonar_data/lines/gt_data.txt";
//
//        std::string pred_file_path = "/home/robot/pointcloud_sonar_data/lines/pred_data.txt";
//
//        std::string pred_file_path1 = "/home/robot/pointcloud_sonar_data/lines/pred_data1.txt";
//
//        std::string pred_file_path2 = "/home/robot/pointcloud_sonar_data/lines/pred_data2.txt";
//
//        std::string pred_file_path3 = "/home/robot/pointcloud_sonar_data/lines/pred_data3.txt";
    };

}
