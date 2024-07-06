#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import radians

joint_angle = 0.0

def joint_state_callback(msg):
    global joint_angle

    joint_name = 'predator/azimuth'
    try:
        joint_index = msg.name.index(joint_name)
    except ValueError:
        rospy.logwarn(f"Joint '{joint_name}' not found in JointState message.")
        return


    joint_angle = msg.position[joint_index]

    rospy.loginfo(f"Joint '{joint_name}' current angle: {joint_angle} radians")


def control_continuous_rotation():
    global joint_angle
    rospy.init_node('control_continuous_rotation')

    joint_position_publisher = rospy.Publisher('/rexrov/predator/azimuth/controller/command', Float64, queue_size=1)


    rate = rospy.Rate(1)

    target_position_degree = 1.0


    rospy.Subscriber('/rexrov/joint_states', JointState, joint_state_callback)

    while not rospy.is_shutdown():

        target_position_radian = radians(target_position_degree)

        joint_position_publisher.publish(target_position_radian)


        rate.sleep()

        if abs(joint_angle) >= 1.57:
            target_position_degree *= -1

if __name__ == "__main__":
    try:
        control_continuous_rotation()
    except rospy.ROSInterruptException:
        pass