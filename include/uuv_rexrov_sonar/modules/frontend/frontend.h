#pragma once

#include "uuv_rexrov_sonar/type/imu.h"
#include "uuv_rexrov_sonar/type/base_type.h"
#include "uuv_rexrov_sonar/type/pose.h"
#include "uuv_rexrov_sonar/type/measure_group.h"
#include "uuv_rexrov_sonar/modules/ieskf/ieskf.h"
#include "uuv_rexrov_sonar/modules/map/rect_map_manager.h"
#include "uuv_rexrov_sonar/modules/frontback_propagate/frontback_propagate.h"
#include "uuv_rexrov_sonar/modules/frontend/lio_zh_model.h"

namespace IESKFSlam
{
    class FrontEnd
    {
    public:
        using Ptr = std::shared_ptr<FrontEnd>;

    private:
        std::deque<IMU> imu_deque;
        std::deque<PointCloud> pointcloud_deque;
        std::shared_ptr<IESKF> ieskf_ptr;
        std::shared_ptr<RectMapManager> map_ptr;
        std::shared_ptr<FrontbackPropagate> fbpropagate_ptr;
        VoxelFilter voxel_filter;
        LIOZHModel::Ptr lio_zh_model_ptr;
        PCLPointCloudPtr filter_point_cloud_ptr, full_point_cloud_ptr;
        bool imu_inited = false;
        double imu_scale = 1;
        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;

    public:
        FrontEnd();
        ~FrontEnd();

        void addImu(const IMU &imu);
        void addPointCloud(const PointCloud &pointcloud);


        bool track();

        const PCLPointCloud &readCurrentPointCloud();
        const PCLPointCloud &readCurrentFullPointCloud();
        bool syncMeasureGroup(MeasureGroup &mg);
        void initState(MeasureGroup &mg);
        IESKF::State18 readState();
    };
}