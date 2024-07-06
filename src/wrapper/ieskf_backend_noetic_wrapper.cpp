#include "uuv_rexrov_sonar/wrapper/ieskf_backend_noetic_wrapper.h"
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic
{
    IESKFBackEndWrapper::IESKFBackEndWrapper(ros::NodeHandle &nh)
    {
        cloud_with_pose_sub = nh.subscribe("/rov/cloud_with_pose", 100, &IESKFBackEndWrapper::cloudWithPoseMsgCallBack, this);
        global_opt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/rov/global_opt_map", 100);

        backend_ptr = std::make_shared<IESKFSlam::BackEnd>();
        ros::spin();
    }

    void IESKFBackEndWrapper::cloudWithPoseMsgCallBack(const uuv_rexrov_sonar::CloudWithPosePtr &msg)
    {

        pcl::fromROSMsg(msg->point_cloud, in_cloud);

        IESKFSlam::Pose pose;
        pose.rotation.x() = msg->pose.orientation.x;
        pose.rotation.y() = msg->pose.orientation.y;
        pose.rotation.z() = msg->pose.orientation.z;
        pose.rotation.w() = msg->pose.orientation.w;
        pose.position.x() = msg->pose.position.x;
        pose.position.y() = msg->pose.position.y;
        pose.position.z() = msg->pose.position.z;
        if (backend_ptr->addFrame(out_cloud, in_cloud, pose))
        {
            publishMsg();
        }
    }

    void IESKFBackEndWrapper::publishMsg()
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(out_cloud, msg);
        msg.header.frame_id = "world";
        global_opt_map_pub.publish(msg);
    }

}