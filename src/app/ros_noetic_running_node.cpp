#include "uuv_rexrov_sonar/wrapper/ieskf_frontend_noetic_wrapper.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"front_end_running_node");
    ros::NodeHandle nh;
    std::shared_ptr<ROSNoetic::IESKFFrontEndWrapper>front_end_ptr;
    front_end_ptr = std::make_shared<ROSNoetic::IESKFFrontEndWrapper>(nh);
    return 0;
}