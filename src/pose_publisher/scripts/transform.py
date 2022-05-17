#!/usr/bin/env python
# license removed for brevity
import roslib 
import rospy
import tf
import actionlib
import turtlesim.msg
import tf.msg
import geometry_msgs.msg
import math
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import time
import os

 
class DynamicTFBroadcaster:
	def __init__(self):
		rospy.loginfo("Initialization")
		rospy.init_node('Tf_generator', anonymous=True)
        	self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)
		self.pose_sub = rospy.Subscriber("/DummyCar_2/pose", PoseStamped, self.tf_callback, queue_size=10)
		self.pose = PoseStamped()
		self.tfm = tf.msg.tfMessage()
		#print(self.pose_sub)

	def tf_callback(self, PoseStamped):		
		self.pose = PoseStamped
		#print(self.pose)
		t = TransformStamped()
		t.header.stamp = self.pose.header.stamp
		t.header.frame_id = "/dummy_map"
		t.child_frame_id = "/dummy_car_2"
		t.transform.translation.x = self.pose.pose.position.x
		t.transform.translation.y = self.pose.pose.position.y
		t.transform.translation.z = self.pose.pose.position.z
 
		t.transform.rotation.x = self.pose.pose.orientation.x
		t.transform.rotation.y = self.pose.pose.orientation.y
		t.transform.rotation.z = self.pose.pose.orientation.z
		t.transform.rotation.w = self.pose.pose.orientation.w
		self.tfm = [t]
		#print(self.pose.pose.position.x)
		self.pub_tf.publish(self.tfm)


if __name__ == '__main__':
	tf = DynamicTFBroadcaster()
	rospy.spin()


