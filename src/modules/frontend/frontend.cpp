#include "uuv_rexrov_sonar/modules/frontend/frontend.h"
#include "pcl/common/transforms.h"

namespace IESKFSlam
{
    FrontEnd::FrontEnd()
    {

        float leaf_size = 0.1f;
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

        ieskf_ptr = std::make_shared<IESKF>();
        map_ptr = std::make_shared<RectMapManager>();
        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();

        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;

        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr, map_ptr->getLocalMap());
    }

    FrontEnd::~FrontEnd()
    {
    }
    void FrontEnd::addImu(const IMU &imu)
    {
        imu_deque.push_back(imu);
        std::cout << "receive imu" << std::endl;
    }

    void FrontEnd::addPointCloud(const PointCloud &pointcloud)
    {
        pointcloud_deque.push_back(pointcloud);
        std::cout << "receive cloud" << std::endl;
    }

    bool FrontEnd::track()
    {
        MeasureGroup mg;
        if (syncMeasureGroup(mg))
        {
            if (!imu_inited)
            {
                map_ptr->reset();
                map_ptr->addScan(mg.cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            std::cout << "sonar start time: " << mg.lidar_begin_time << " sonar end time: " << mg.lidar_end_time << std::endl;
            fbpropagate_ptr->propagate(mg, ieskf_ptr);
            // voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
            // voxel_filter.filter(*filter_point_cloud_ptr);

            full_point_cloud_ptr = mg.cloud.cloud_ptr;

            filter_point_cloud_ptr = mg.cloud.cloud_ptr;
            ieskf_ptr->update();
            auto x = ieskf_ptr->getX();
            std::cout << "update sucess! " << std::endl;
            map_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position);
            return true;
        }
        return false;
    }

    const PCLPointCloud &FrontEnd::readCurrentPointCloud()
    {
        return *filter_point_cloud_ptr;
    }

    const PCLPointCloud &FrontEnd::readCurrentFullPointCloud()
    {
        return *full_point_cloud_ptr;
    }

    bool FrontEnd::syncMeasureGroup(MeasureGroup &mg)
    {
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();

        if (pointcloud_deque.empty() || imu_deque.empty() || pointcloud_deque.size() < 2)
        {
            return false;
        }

        //. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time = pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque.at(1).time_stamp.sec();

        if (imu_end_time < cloud_end_time)
        {
            return false;
        }

        if (cloud_end_time < imu_start_time)
        {

            pointcloud_deque.pop_front();
            return false;
        }

        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;

        while (!imu_deque.empty())
        {
            if (imu_deque.front().time_stamp.sec() < mg.lidar_end_time)
            {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
            }
            else
            {
                break;
            }
        }

        if (mg.imus.size() <= 5)
        {

            return false;
        }
        return true;
    }

    void FrontEnd::initState(MeasureGroup &mg)
    {
        static int imu_count = 0;
        static Eigen::Vector3d mean_acc{0, 0, 0};
        auto &ieskf = *ieskf_ptr;
        if (imu_inited)
        {
            return;
        }

        for (size_t i = 0; i < mg.imus.size(); i++)
        {
            imu_count++;
            auto x = ieskf.getX();
            mean_acc += mg.imus[i].acceleration;
            x.bg += mg.imus[i].gyroscope;
            ieskf.setX(x);
        }
        if (imu_count >= 5)
        {
            auto x = ieskf.getX();
            mean_acc /= double(imu_count);

            x.bg /= double(imu_count);
            imu_scale = GRAVITY / mean_acc.norm();
            fbpropagate_ptr->imu_scale = imu_scale;
            fbpropagate_ptr->last_imu = mg.imus.back();

            x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
            ieskf.setX(x);
            imu_inited = true;
        }
        return;
    }
    IESKF::State18 FrontEnd::readState()
    {
        return ieskf_ptr->getX();
    }

}
