#pragma once
#include "uuv_rexrov_sonar/modules/ieskf/ieskf.h"
#include "uuv_rexrov_sonar/type/base_type.h"
#include "uuv_rexrov_sonar/math/geometry.h"
#include "uuv_rexrov_sonar/math/SO3.hpp"

namespace IESKFSlam
{
    class LIOZHModel : public IESKF::CalcZHInterface
    {
    private:
        const int NEAR_POINTS_NUM = 5;
        // 1) point_imu 2)normal_vector 3)distance
        using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double>;
        KDTreeConstPtr global_map_kdtree_ptr;
        PCLPointCloudPtr current_cloud_ptr;
        PCLPointCloudConstPtr local_map_ptr;

    public:
        using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(KDTreeConstPtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map)
        {
            global_map_kdtree_ptr = kd_tree;
            current_cloud_ptr = current_cloud;
            local_map_ptr = local_map;
        }
        bool calculate(const IESKF::State18 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) override
        {
            std::vector<loss_type> loss_v;
            loss_v.resize(current_cloud_ptr->size());
            std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);
            std::vector<loss_type> loss_real;
            int vaild_points_num = 0;

            for (size_t i = 0; i < current_cloud_ptr->size(); i++)
            {
                Point point_imu = current_cloud_ptr->points[i];
                Point point_world;
                point_world = transformPoint(point_imu, state.rotation, state.position);

                std::vector<int> point_ind;
                std::vector<float> distance;
                global_map_kdtree_ptr->nearestKSearch(point_world, NEAR_POINTS_NUM, point_ind, distance);

                if (distance.size() < NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM - 1] > 5)
                {
                    continue;
                }

                std::vector<Point> planar_points;
                for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
                {
                    planar_points.push_back(local_map_ptr->at(point_ind[ni]));
                }
                Eigen::Vector4d pabcd;

                if (planarCheck(planar_points, pabcd, 0.1))
                {

                    double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);


                    loss_type loss;
                    loss.thrid = pd;
                    loss.first = {point_imu.x, point_imu.y, point_imu.z};
                    loss.second = pabcd.block<3, 1>(0, 0);
                    if (isnan(pd) || isnan(loss.second(0)) || isnan(loss.second(1)) || isnan(loss.second(2)))
                        continue;

                    double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
                    if (s > 0.9)
                    {

                        vaild_points_num++;
                        loss_v[i] = (loss);
                        is_effect_point[i] = true;
                    }
                }
            }
            for (size_t i = 0; i < current_cloud_ptr->size(); i++)
            {
                if (is_effect_point[i])
                    loss_real.push_back(loss_v[i]);
            }

            vaild_points_num = loss_real.size();
            H = Eigen::MatrixXd::Zero(vaild_points_num, 18);
            Z.resize(vaild_points_num, 1);
            for (int vi = 0; vi < vaild_points_num; vi++)
            {

                Eigen::Vector3d dr = -1 * loss_real[vi].second.transpose() * state.rotation.toRotationMatrix() * skewSymmetric(loss_real[vi].first);
                H.block<1, 3>(vi, 0) = dr.transpose();
                H.block<1, 3>(vi, 3) = loss_real[vi].second.transpose();

                Z(vi, 0) = loss_real[vi].thrid;
            }
            return true;
        }
    };

}