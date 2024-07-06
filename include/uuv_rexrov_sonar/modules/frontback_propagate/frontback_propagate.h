#pragma once

#include "uuv_rexrov_sonar/modules/ieskf/ieskf.h"
#include "uuv_rexrov_sonar/type/measure_group.h"
namespace IESKFSlam
{
    class FrontbackPropagate
    {
    private:
    public:
        double imu_scale;
        IMU last_imu;
        FrontbackPropagate(/* args */);
        ~FrontbackPropagate();
        void propagate(MeasureGroup &mg, IESKF::Ptr ieskf_ptr);
    };

}
