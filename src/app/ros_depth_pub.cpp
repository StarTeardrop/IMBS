#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"


sensor_msgs::FluidPressure depth_fluidpressue;

void Sub_Rov_Depth(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    depth_fluidpressue = *msg;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iskfslam_depth_node");

    ros::NodeHandle nh;


    ros::Subscriber rov_depth_sub = nh.subscribe<sensor_msgs::FluidPressure>("/rexrov1/pressure", 100, Sub_Rov_Depth);


    ros::Publisher rov_depth_pub = nh.advertise<sensor_msgs::FluidPressure>("/rov/depth", 100);


    ros::Rate rate(50);

    while(ros::ok())
    {

        rov_depth_pub.publish(depth_fluidpressue);


        rate.sleep();
        

        ros::spinOnce();
    }
    return 0;
}
