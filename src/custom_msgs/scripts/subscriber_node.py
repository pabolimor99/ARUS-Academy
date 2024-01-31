#!/usr/bin/env python3
import rospy
from custom_msgs.msg import Cone, Map


def callback(msg):
    for cone in msg.cones:
        x= cone.position.x
        y= cone.position.y
        z= cone.position.z
        color= cone.color
        confidence= cone.confidence

        rospy.loginfo(f'[{x}, {y}, {z}, {color}, {confidence}]')



def listener():
    rospy.init_node('Subscriber_Node')
    rospy.Subscriber('perception_map', Map, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()