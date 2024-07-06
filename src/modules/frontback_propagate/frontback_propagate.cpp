#include "uuv_rexrov_sonar/modules/frontback_propagate/frontback_propagate.h"

namespace IESKFSlam
{

    FrontbackPropagate::FrontbackPropagate(/* args */)
    {
    }

    FrontbackPropagate::~FrontbackPropagate()
    {
    }

    void FrontbackPropagate::propagate(MeasureGroup &mg, IESKF::Ptr ieskf_ptr)
    {
        mg.imus.push_front(last_imu);
        double dt = 0;
        IMU in;

        IESKF::State18 imu_state;
        for (auto it_imu = mg.imus.begin(); it_imu < (mg.imus.end() - 1); it_imu++)
        {
            auto &&head = *(it_imu);
            auto &&tail = *(it_imu + 1);
            auto angvel_avr = 0.5 * (head.gyroscope + tail.gyroscope);
            auto acc_avr = 0.5 * (head.acceleration + tail.acceleration) * imu_scale;
            double dt = tail.time_stamp.sec() - head.time_stamp.sec();
            in.acceleration = acc_avr;
            in.gyroscope = angvel_avr;
            ieskf_ptr->predict(in, dt);
        }


        dt = mg.lidar_end_time - mg.imus.back().time_stamp.sec();
        ieskf_ptr->predict(in, dt);

        last_imu = mg.imus.back();

        last_imu.time_stamp.fromSec(mg.lidar_end_time);
    }
}
