#pragma once

#include "uuv_rexrov_sonar/type/pointcloud.h"
#include "uuv_rexrov_sonar/type/base_type.h"

namespace IESKFSlam
{
    class RectMapManager
    {
    private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;

    public:
        RectMapManager();
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap();

        KDTreeConstPtr readKDtree();
    };

}