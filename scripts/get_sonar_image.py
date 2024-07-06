#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import cv2

start_time = 0.0

def image_callback(data):
    global start_time
    try:

        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return
    current_time = rospy.get_time() - start_time
    current_time_str = "{:.3f}".format(current_time)

    image_filename = f"sonar_images/{current_time_str}.jpg"
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo(f"save image in {current_time_str}")


if __name__ == "__main__":
    rospy.init_node('get_sonar_image')


    rospy.Subscriber("/rexrov1/blueview_p900/sonar_image_new", Image, image_callback)

    start_time = rospy.get_time()

    rospy.spin()