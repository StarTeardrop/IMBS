#include "uuv_rexrov_sonar/modules/backend/backend.h"

namespace IESKFSlam
{
    BackEnd::BackEnd()
    {
        extrin_r.setIdentity();
        extrin_t.setZero();
        // voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    }

    bool BackEnd::addFrame(PCLPointCloud &opt_map, PCLPointCloud &cloud, Pose &pose)
    {

        static int cnt = 0;
        if (cnt > 10 || poses.empty() || (poses.back().position - pose.position).norm() > 1)
        {
            sc_manager.makeAndSaveScancontextAndKeys(cloud);
            clouds.push_back(cloud);
            poses.push_back(pose);
            cnt = 0;
            auto res = sc_manager.detectLoopClosureID();


            if (res.first != -1)
            {

                Eigen::Matrix4f trans_icp;
                Eigen::Matrix4d T_L_I, T_I_L, T_c;
                if (scanRegister(trans_icp, clouds.size() - 1, res.first, res.second))
                {

                    BinaryEdge be;
                    be.from_vertex = cloud.size() - 1;
                    be.to_vertex = res.first;
                    T_L_I.setIdentity();
                    T_L_I.block<3, 3>(0, 0) = extrin_r.toRotationMatrix();
                    T_L_I.block<3, 1>(0, 3) = extrin_t;

                    T_I_L.block<3, 3>(0, 0) = extrin_r.conjugate().toRotationMatrix();
                    T_I_L.block<3, 1>(0, 3) = extrin_r.conjugate() * extrin_t * (-1.0);

                    T_c = T_L_I * trans_icp.cast<double>() * T_I_L;
                    be.constraint.rotation = T_c.block<3, 3>(0, 0);
                    be.constraint.position = T_c.block<3, 1>(0, 3);
                    binary_edges.push_back(be);

                    std::vector<Pose> copy_poses = poses;
                    if (pgo.slove(copy_poses, binary_edges))
                    {
                        opt_map.clear();
                        for (int i = 0; i < clouds.size(); i++)
                        {
                            Eigen::Matrix4f T_I_W, T_L_W;
                            T_I_W.setIdentity();
                            T_I_W.block<3, 3>(0, 0) = copy_poses[i].rotation.cast<float>().toRotationMatrix();
                            T_I_W.block<3, 1>(0, 3) = copy_poses[i].position.cast<float>();
                            T_L_W = T_I_W * T_L_I.cast<float>();
                            PCLPointCloud global_cloud;
                            // voxel_filter.setInputCloud(clouds[i].makeShared());
                            // voxel_filter.filter(global_cloud);
                            pcl::transformPointCloud(clouds[i], global_cloud, T_L_W);
                            opt_map += global_cloud;
                        }
                        voxel_filter.setInputCloud(opt_map.makeShared());
                        voxel_filter.filter(opt_map);
                        // update pose
                        for (int i = 0; i < poses.size(); i++)
                        {
                            poses[i] = copy_poses[i];
                        }

                        return true;
                    }
                    else
                    {
                        binary_edges.pop_back();
                    }
                }
            }
        }
        cnt++;
        return false;
    }

    bool BackEnd::scanRegister(Eigen::Matrix4f &match_result, int from_id, int to_id, float angle)
    {

        Eigen::AngleAxisf init_rotation(-1 * angle, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();

        initial_transform.block<3, 3>(0, 0) = init_rotation.toRotationMatrix();

        pcl::IterativeClosestPoint<Point, Point> icp;
        icp.setInputSource(clouds[from_id].makeShared());
        icp.setInputTarget(clouds[to_id].makeShared());

        icp.setMaximumIterations(20);
        PCLPointCloud result;
        icp.align(result, initial_transform);

        match_result = icp.getFinalTransformation();
        std::cout << "icp score: " << icp.getFitnessScore() << std::endl;

        return icp.getFitnessScore() < 3.0;
    }

}