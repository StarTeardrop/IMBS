#pragma once
#include "uuv_rexrov_sonar/modules/pose_graph_opt/pose_graph_opt.h"
#include "uuv_rexrov_sonar/type/base_type.h"
#include "uuv_rexrov_sonar/type/pointcloud.h"
#include "uuv_rexrov_sonar/type/pose.h"
#include <pcl/registration/icp.h>
#include "scan_context/Scancontext.h"

namespace IESKFSlam
{
    class BackEnd
    {
    private:
        SC2::SCManager sc_manager;

        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;

        // . 关键帧位姿：
        std::vector<Pose> poses;
        std::vector<PCLPointCloud> clouds;
        std::vector<BinaryEdge> binary_edges;
        VoxelFilter voxel_filter;
        PoseGraphOpt pgo;

    public:
        using Ptr = std::shared_ptr<BackEnd>;

        BackEnd();
        bool addFrame(PCLPointCloud &opt_map, PCLPointCloud &cloud, Pose &pose);
        bool scanRegister(Eigen::Matrix4f &match_result, int from_id, int to_id, float angle);
    };

};
