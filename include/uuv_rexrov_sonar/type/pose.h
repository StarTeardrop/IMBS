#pragma once

#include <Eigen/Dense>
#include "timestamp.h"
namespace IESKFSlam
{
    struct Pose
    {
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
} 
