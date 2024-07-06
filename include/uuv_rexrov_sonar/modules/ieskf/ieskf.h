#pragma once

#include <Eigen/Dense>
#include <memory>
#include "uuv_rexrov_sonar/type/imu.h"

namespace IESKFSlam
{
    class IESKF
    {
    public:
        using Ptr = std::shared_ptr<IESKF>;
        struct State18
        {
            Eigen::Quaterniond rotation;
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d gravity;
            State18()
            {
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d::Zero();
                bg = Eigen::Vector3d::Zero();
                ba = Eigen::Vector3d::Zero();
                gravity = Eigen::Vector3d::Zero();
            }
        };

        class CalcZHInterface
        {
        public:
            virtual bool calculate(const State18 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) = 0;
        };

        std::shared_ptr<CalcZHInterface> calc_zh_ptr;

    private:
        State18 X;
        Eigen::Matrix<double, 18, 18> P;
        Eigen::Matrix<double, 12, 12> Q;
        int iter_times = 10;

    public:
        IESKF();
        ~IESKF();
        void predict(IMU imu, double dt);
        bool update();
        const State18 &getX();
        void setX(const State18 &x_in);
        Eigen::Matrix<double, 18, 1> getErrorState18(const State18 &s1, const State18 &s2);
    };
}