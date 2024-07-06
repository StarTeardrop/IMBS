#pragma once
#include "uuv_rexrov_sonar/type/pose.h"
#include <ceres/ceres.h>
namespace IESKFSlam
{
class PoseGraph3dErrorTerm
{
  public:

    PoseGraph3dErrorTerm(Pose constraint, Eigen::Matrix<double, 6, 6> input_sqrt_information)
        : q_constraint(constraint.rotation), p_constraint(constraint.position),
          sqrt_information(input_sqrt_information){};

    template <typename T>
    bool operator()(const T *const p_a_ptr, const T *const q_a_ptr, const T *const p_b_ptr, const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_relative = q_a_inverse * q_b;

        Eigen::Matrix<T, 3, 1> p_ab_relative = q_a_inverse * (p_b - p_a);
        Eigen::Quaternion<T> delta_q = q_constraint.cast<T>() * q_ab_relative.conjugate();
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_ab_relative - p_constraint.cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
        sqrt_information.cast<T>();
        residuals = sqrt_information.cast<T>() * residuals;
        return true;
        // Eigen::Quaternion
    }

    static ceres::CostFunction *Create(
        const Pose &constraint,
        const Eigen::Matrix<double, 6, 6> &input_sqrt_information = Eigen::Matrix<double, 6, 6>::Identity())
    {
        return new ceres::AutoDiffCostFunction
            <PoseGraph3dErrorTerm, 6, 3, 4, 3,
             4>
            (new PoseGraph3dErrorTerm(constraint, input_sqrt_information));
    }

  private:
    Eigen::Quaterniond q_constraint;
    Eigen::Vector3d p_constraint;
    Eigen::Matrix<double, 6, 6> sqrt_information;
};

} // namespace IESKFSlam
