#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SonarImageFlipNode:
    def __init__(self, uuv_name):


        self.image_sub = rospy.Subscriber(uuv_name + "/blueview_p900/sonar_image", Image, self.image_callback)


        self.image_pub = rospy.Publisher(uuv_name +"/blueview_p900/sonar_image_new", Image, queue_size=10)


        self.bridge = CvBridge()
        
    def image_callback(self, data):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))
            return


        flipped_image = cv2.flip(cv_image, 1)


        flipped_ros_image = self.bridge.cv2_to_imgmsg(flipped_image, "bgr8")


        self.image_pub.publish(flipped_ros_image)

if __name__ == '__main__':

        rospy.init_node('sonar_image_flip_node', anonymous=True)


        uuv_name = rospy.get_param("~uuv_name")
        

        node = SonarImageFlipNode(uuv_name)
        
        rospy.spin()





