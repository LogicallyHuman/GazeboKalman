#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import math

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(30) # 10hz
count = 0
while not rospy.is_shutdown():
    count += 1
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = math.cos(count / 10.0) 
    marker.color.g = math.sin(count / 10.0) 
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = math.cos(count / 10.0)
    marker.pose.position.y = math.sin(count / 10.0)
    marker.pose.position.z = 0 


    pub.publish(marker)
    rate.sleep()