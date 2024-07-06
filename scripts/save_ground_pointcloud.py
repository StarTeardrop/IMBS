#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import os


file_path = '/home/robot/pointcloud_sonar_data/ground_pointcloud/pointcloud.txt'


pointcloud_data = []


tf_buffer = None

def cloud_callback(cloud_msg):
    global pointcloud_data, tf_buffer
    try:

        transform = tf_buffer.lookup_transform("world", cloud_msg.header.frame_id,
                                               rospy.Time(0), rospy.Duration(1.0))
        

        transformed_cloud = do_transform_cloud(cloud_msg, transform)
        

        points = list(pc2.read_points(transformed_cloud, skip_nans=True))
        pointcloud_data.extend(points)
    except Exception as e:
        rospy.logerr(f"Failed to process point cloud: {e}")

def save_pointcloud_data():
    global pointcloud_data
    try:
        with open(file_path, 'w') as file:
            for point in pointcloud_data:

                x, y, z = point[:3]
                file.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        rospy.loginfo("Saved pointcloud data to file.")
    except Exception as e:
        rospy.logerr(f"Failed to write to file: {e}")

def timer_callback(event):
    save_pointcloud_data()

if __name__ == "__main__":
    rospy.init_node('pointcloud_saver', anonymous=True)
    

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    

    rospy.Subscriber('/rov/lidar', PointCloud2, cloud_callback)
    

    rospy.Timer(rospy.Duration(10), timer_callback)
    
    rospy.spin()
