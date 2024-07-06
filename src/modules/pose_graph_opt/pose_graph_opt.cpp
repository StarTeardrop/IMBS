#include "uuv_rexrov_sonar/modules/pose_graph_opt/pose_graph_opt.h"
#include "uuv_rexrov_sonar/modules/pose_graph_opt/pose_graph_3d_error_term.h"
#include <ceres/ceres.h>

namespace IESKFSlam
{
    PoseGraphOpt::PoseGraphOpt(/* args */)
    {
    }

    bool PoseGraphOpt::slove(std::vector<Pose> &poses, std::vector<BinaryEdge> &bes)
    {
        if (poses.size() <= 10)
            return false;


        ceres::Problem *problem = new ceres::Problem();

        ceres::LossFunction *loss_function = nullptr;

        ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold();

        for (int i = 0; i < poses.size() - 1; i++)
        {
            Pose constraint;

            constraint.rotation = poses[i].rotation.conjugate() * poses[i + 1].rotation;
            constraint.position = poses[i].rotation.conjugate() * (poses[i + 1].position - poses[i].position);


            ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(constraint);

            problem->AddResidualBlock(cost_function, loss_function, poses[i].position.data(),
                                      poses[i].rotation.coeffs().data(), poses[i + 1].position.data(),
                                      poses[i + 1].rotation.coeffs().data());

            problem->SetManifold(poses[i].rotation.coeffs().data(), quaternion_manifold);
            problem->SetManifold(poses[i + 1].rotation.coeffs().data(), quaternion_manifold);
        }


        problem->SetParameterBlockConstant(poses.front().rotation.coeffs().data());
        problem->SetParameterBlockConstant(poses.front().position.data());



        for (auto &&be : bes)
        {

            ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(be.constraint);

            problem->AddResidualBlock(cost_function, loss_function, poses[be.to_vertex].position.data(),
                                      poses[be.to_vertex].rotation.coeffs().data(), poses[be.from_vertex].position.data(),
                                      poses[be.from_vertex].rotation.coeffs().data());

            problem->SetManifold(poses[be.to_vertex].rotation.coeffs().data(), quaternion_manifold);
            problem->SetManifold(poses[be.from_vertex].rotation.coeffs().data(), quaternion_manifold);
        }

        ceres::Solver::Options options;

        options.max_num_iterations = 200;

        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        ceres::Solver::Summary summary;

        ceres::Solve(options, problem, &summary);

        std::cout << summary.BriefReport() << std::endl;
        return summary.termination_type == ceres::CONVERGENCE;
    }
}
