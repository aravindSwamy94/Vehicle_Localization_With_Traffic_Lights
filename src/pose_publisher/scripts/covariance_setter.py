#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
import numpy as np
import math


class CovarianceSetter():

    def __init__(self):
        """
        This node assigns the missing covariance values to the measurements required by the ekf
        """

        self.init_node = rospy.init_node('covariance_setter', anonymous=True)

        self.imu = 'imu'
        self.wheel_odom = 'wheel_odom'
	self.gps = 'pose_gps'

        # extract the ros parameters
        self.imu_topic = rospy.get_param('covariance_setter/imu/topic', 'imu/data')
        self.imu_rate = rospy.get_param('covariance_setter/imu/rate', 60.)
        self.imu_msgType = rospy.get_param('covariance_setter/imu/msgType', 'Imu')
        self.imu_covMatrix = rospy.get_param('covariance_setter/imu/covariance_matrix', None)

        self.wheel_odom_topic = rospy.get_param('covariance_setter/wheel_odom/topic', 'wheel_odom')
        self.wheel_odom_rate = rospy.get_param('covariance_setter/wheel_odom/rate', 60.)
        self.wheel_odom_msgType = rospy.get_param('covariance_setter/wheel_odom/msgType', 'Odometry')
        self.wheel_odom_covMatrix = rospy.get_param('covariance_setter/wheel_odom/covariance_matrix', None)
	
	self.gps_topic = rospy.get_param('covariance_setter/gps/topic', 'pose_gps')
        self.gps_rate = rospy.get_param('covariance_setter/gps/rate', 60.)
        self.gps_msgType = rospy.get_param('covariance_setter/gps/msgType', 'PoseWithCovarianceStamped')
        self.gps_covMatrix = rospy.get_param('covariance_setter/gps/covariance_matrix', None)

        # initialize subscribers
        self.subber_imu = rospy.Subscriber(self.imu_topic, eval(self.imu_msgType), self.callback, (self.imu), queue_size=3)
        self.subber_wheel_odom = rospy.Subscriber(self.wheel_odom_topic, eval(self.wheel_odom_msgType), self.callback,  (self.wheel_odom), queue_size=3)
        self.subber_gps = rospy.Subscriber(self.gps_topic, eval(self.gps_msgType), self.callback,  (self.gps), queue_size=3)

        # initialize publisher dict
        self.publer = {}
        # create publisher as dictionary entries so they publish automatically during sensor callback
        self.publer[self.imu] = rospy.Publisher('/imu/data_withCov', eval(self.imu_msgType), queue_size=7)
        self.publer[self.wheel_odom] = rospy.Publisher('/wheel_odom_withCov', eval(self.wheel_odom_msgType), queue_size=7)
        self.publer[self.gps] = rospy.Publisher('/pose_gps_withCov', eval(self.gps_msgType), queue_size=7)

    def callback(self, msg, source):
        """
        callback function for all subscribers. it will assign the respective covariances to the received message
        based on msg type
        :param msg: received sensor msg
        :param source: string for received sensor msg type to select the correct publisher
        :return:
        """
        # retrieve msg type of received msg
        type = msg._type

        # in case the received msg is odometry
        if 'Odometry' in type:

            # assign covs to the ros msg
            msg.pose.covariance, msg.twist.covariance = self.assign_odometry_cov(msg, self.wheel_odom_covMatrix)

            # publish original msg with adjusted covs and correct linear x velocity
            self.publer[source].publish(msg)
            #print('Covariance set and published for {}'.format(source))

        elif 'Imu' in type:
            # assign covs to the ros msg
            msg.orientation_covariance, msg.angular_velocity_covariance, \
            msg.linear_acceleration_covariance = self.assign_imu_cov(msg, self.imu_covMatrix)
            # publish original msg with adjusted covs
            self.publer[source].publish(msg)
            #print('Covariance set and published for {}'.format(source))

	elif 'PoseWithCovarianceStamped' in type:
            msg.pose.covariance = self.assign_gps_cov(msg, self.gps_covMatrix)
            self.publer[source].publish(msg)


    def assign_imu_cov(self, imu_msg, covMatrix):
        """
        assign values to the covariance entries for ros msgs of type Imu
        :return: covariance lists for the imu entries
        """
        # init list which will hold the covariance matrix
        covariance_orientation = [0.1] * 9
        covariance_angular_velocity = [0.1] * 9
        covariance_linear_acceleration = [0.1] * 9

        # return the init covariances if config file does not specify cov values
        if covMatrix is None:
            return covariance_orientation, covariance_angular_velocity, covariance_linear_acceleration

        # assign the covariance values if specified in config
        if covMatrix['orientation'] is not None and len(covMatrix['orientation']) == 9:

            covariance_orientation = covMatrix['orientation']

        elif len(covMatrix['orientation']) != 9:
            self.print_wrongSize('orientation')

        if covMatrix['angular_velocity'] is not None and len(covMatrix['angular_velocity']) == 9:

            covariance_angular_velocity = covMatrix['angular_velocity']

        elif len(covMatrix['angular_velocity']) != 9:
            self.print_wrongSize('angular_velocity')

        if covMatrix['linear_acceleration'] is not None and len(covMatrix['linear_acceleration']) == 9:

            covariance_linear_acceleration = covMatrix['linear_acceleration']

        elif len(covMatrix['linear_acceleration']) != 9:
            self.print_wrongSize('linear_acceleration')

        return covariance_orientation, covariance_angular_velocity, covariance_linear_acceleration

    def assign_odometry_cov(self, odom_msg, covMatrix):
        """
        assign values to the covariance entries for ros msgs of type Odometry
        :param odom_msg: odometry message
        :param covMatrix: list entries holding the to be assigned covariance values
        :return: covariance lists for the odometry entries
        """
        # init list which will hold the covariance matrix
        covariance_pose = [0.1] * 36  # pose msg holds 36 covariance elements
        covariance_twist = [0.1] * 36  # twist msg holds 36 covariance elements

        # if no covariance matrix was defined
        if covMatrix is None:
            return covariance_pose, covariance_twist

        # if config yaml contains the covariance values required
        if covMatrix['pose'] is not None and len(covMatrix['pose']) == 36:
            covariance_pose = covMatrix['pose']
        elif len(covMatrix['pose']) != 36:
            self.print_wrongSize('pose')

        if covMatrix['twist'] is not None and len(covMatrix['twist']) == 36:
            covariance_twist = covMatrix['twist']
        elif len(covMatrix['twist']) != 36:
            self.print_wrongSize('twist')

        return covariance_pose, covariance_twist

    def assign_gps_cov(self, gps_msg, covMatrix):
        """
        assign values to the covariance entries for pose related ros msg types
        :return: covariance list for pose entries
        """
        # init list which will hold the covariance matrix
        if covMatrix is None:
            covariance_pose = [0.1] * 36  # pose msg holds 36 covariance elements
        # if config yaml contains the covariance values required
        else:
            covariance_pose = covMatrix['pose']

        return covariance_pose

    def print_wrongSize(self, type):
        """
        error message print
        :param type: covariance type
        :return:
        """
        print("Wrong size for Covariance Matrix of type {} specified in config file. Initial values are used instead.".format(type))


if __name__ == '__main__':
    cov_setter = CovarianceSetter()
    rospy.spin()
