#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.msg import *
import time
import os

class odom_publisher:
   	def __init__(self):      
		# Create publisher
		rospy.loginfo("Initialization")
		rospy.init_node('Pose_generator', anonymous=True)
		self.message_pub = rospy.Publisher("/pose_gps", PoseWithCovarianceStamped, queue_size=10)
		self.pose = PoseWithCovarianceStamped()
		self.twist = NavSatFix()	
		rospy.Subscriber("/gps/data", NavSatFix, self.twist_callback, queue_size=10)

   	def twist_callback(self, NavSatFix):
		self.twist = NavSatFix
		self.pose.header = self.twist.header
		self.pose.pose.pose.position.x = self.twist.latitude
		self.pose.pose.pose.position.y = self.twist.longitude
		self.pose.pose.pose.position.z = self.twist.altitude
	   	self.message_pub.publish(self.pose)

# If the python node is executed as main process (sourced directly)
def main(args):
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        lp = odom_publisher()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

if __name__ == '__main__':
    main(sys.argv)
	   





