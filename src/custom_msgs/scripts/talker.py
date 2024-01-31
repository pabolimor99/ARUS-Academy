#!/usr/bin/env python
import rospy
from custom_msgs.msg import Cone, Map

pub= rospy.Publisher()
rospy.init_node()
r= rospy.Rate(10)

msg= Cone()
'''
MESSAGE CONTENT
'''

while not rospy.is_shutdown():
    rospy.publish(msg)
    r.sleep()