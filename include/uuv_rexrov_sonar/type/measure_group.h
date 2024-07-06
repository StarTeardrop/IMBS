#pragma once
#include "uuv_rexrov_sonar/type/imu.h"
#include "uuv_rexrov_sonar/type/pointcloud.h"
#include <deque>
namespace IESKFSlam
{
    struct MeasureGroup
    {
        double lidar_begin_time;
        std::deque<IMU> imus;
        PointCloud cloud;
        double lidar_end_time;
    };
}