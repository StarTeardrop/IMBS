#pragma once
#include "uuv_rexrov_sonar/CloudWithPose.h"
#include "uuv_rexrov_sonar/modules/backend/backend.h"
#include <ros/ros.h>

namespace ROSNoetic
{
    class IESKFBackEndWrapper
    {
    private:
        ros::Publisher global_opt_map_pub;
        ros::Subscriber cloud_with_pose_sub;
        IESKFSlam::PCLPointCloud in_cloud, out_cloud;
        // function
        IESKFSlam::BackEnd::Ptr backend_ptr;

        void cloudWithPoseMsgCallBack(const uuv_rexrov_sonar::CloudWithPosePtr &msg);
        void publishMsg();

    public:
        IESKFBackEndWrapper(ros::NodeHandle &nh);
    };
}