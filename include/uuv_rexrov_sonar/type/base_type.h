#pragma once

#include "uuv_rexrov_sonar/type/pointcloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
namespace IESKFSlam
{

    using VoxelFilter = pcl::VoxelGrid<Point>;

    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    using KDTreeConstPtr = KDTree::ConstPtr;


    const double GRAVITY = 9.81;
    template<typename _first, typename _second, typename _thrid>
    struct triple{
        _first first;
        _second second;
        _thrid thrid;
    };
} 