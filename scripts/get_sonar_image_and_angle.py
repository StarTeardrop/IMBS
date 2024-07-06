#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2
from math import radians, degrees

# 全局变量
joint_angle_rad = 0.0
start_time = 0.0
degree_interval = 1.0
cv_image = None

def joint_state_callback(msg):
    global joint_angle_rad

    joint_name = 'predator/azimuth'
    try:
        joint_index = msg.name.index(joint_name)
    except ValueError:
        rospy.logwarn(f"Joint '{joint_name}' not found in JointState message.")
        return


    current_joint_angle_rad = msg.position[joint_index]


    delta_angle_rad = abs(current_joint_angle_rad - joint_angle_rad)

    if degrees(delta_angle_rad) >= degree_interval:
        joint_angle_rad = current_joint_angle_rad
        rospy.loginfo(f"Joint '{joint_name}' current angle: {degrees(joint_angle_rad)} degrees.")
        save_image()

def save_image():
    global joint_angle_rad, start_time, cv_image
    if cv_image is not None:

        current_time = rospy.get_time() - start_time
        current_time_str = "{:.3f}".format(current_time)

        joint_angle_deg = degrees(joint_angle_rad)
        image_filename = f"coffe_table_sonar_images/sonar_image+{current_time_str}+{joint_angle_deg:.3f}.jpg"
        cv2.imwrite(image_filename, cv_image)


def image_callback(data):
    global cv_image
    try:

        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return
    
    

if __name__ == "__main__":
    rospy.init_node('get_sonar_image_and_angle')


    rospy.Subscriber('/rexrov/joint_states', JointState, joint_state_callback)


    rospy.Subscriber("/rexrov/blueview_p900/sonar_image_new", Image, image_callback)


    start_time = rospy.get_time()

    rospy.spin()



