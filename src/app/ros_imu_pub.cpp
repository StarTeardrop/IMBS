#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


sensor_msgs::Imu imu;

void Sub_Rov_Imu(const sensor_msgs::Imu::ConstPtr& msg_imu)
{

    imu = *msg_imu;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iskfslam_imu_node");
    ros::NodeHandle nh;


    ros::Subscriber rov_imu_sub = nh.subscribe<sensor_msgs::Imu>("/rexrov1/imu",100,Sub_Rov_Imu);


    ros::Publisher rov_imu_pub = nh.advertise<sensor_msgs::Imu>("/rov/origin_imu", 100);
    

    ros::Rate r(50);


    while (ros::ok())
    {

        rov_imu_pub.publish(imu);


        r.sleep();
        

        ros::spinOnce();

    }


    return 0;
}

/**
 * topic:
 * /rexrov1/imu
 * Type: sensor_msgs/Imu
 *
 *
 *
 * topic:
 * /rexrov1/blueview_p900/sonar_image_new
 * Type: sensor_msgs/Image
 *
 */