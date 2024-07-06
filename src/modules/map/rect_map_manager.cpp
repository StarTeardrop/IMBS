#include "uuv_rexrov_sonar/modules/map/rect_map_manager.h"
#include "pcl/common/transforms.h"
#include "uuv_rexrov_sonar/math/math.hpp"

namespace IESKFSlam
{

    RectMapManager::RectMapManager()
    {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
    }

    RectMapManager::~RectMapManager()
    {
    }

    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d &pos_t)
    {
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan, scan, compositeTransform(att_q, pos_t).cast<float>());
        *local_map_ptr += scan;

        kdtree_ptr->setInputCloud(local_map_ptr);
    }

    void RectMapManager::reset()
    {
        local_map_ptr->clear();
    }
    PCLPointCloudConstPtr RectMapManager::getLocalMap()
    {
        return local_map_ptr;
    }

    KDTreeConstPtr RectMapManager::readKDtree(){
        return kdtree_ptr;
    }

}
