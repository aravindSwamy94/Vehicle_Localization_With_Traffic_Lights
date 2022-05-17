#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
import cv2
from cv_bridge import CvBridge
import tf
import tf2_ros
from custom_obstacle_msgs.msg import Position
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import message_filters
import math
import numpy as np
import pylab as plt

class Server:
    def __init__(self):
        self.bridge = CvBridge()
        self.robot_position_topic = '/DummyCar_2/pose'
        self.depth_image_topic = '/DummyCar2/dummy2_car/depth/image'
        self.depth_camera_info_topic = '/DummyCar2/dummy2_car/depth/camera_info'
        self.bounding_box_topic = '/darknet_ros_car/bounding_boxes'
        self.publish_ego_world_coord = rospy.Publisher("/pose_traffic_car", PoseWithCovarianceStamped, queue_size=10)        


    def iConvertTo360(self,x):
        if x > 0:
            x+=0.0
        else:
            x=3.14+x
        return x


    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth, cameraInfo):
        Xreal = ((x-cameraInfo.K[2])*depth)/cameraInfo.K[0]
        Yreal = ((y-cameraInfo.K[5])*depth)/cameraInfo.K[4]
        result = [Xreal, Yreal, depth]
        return result



    def callback(self, image, info, boxc, robo_pose):
        world_points = []
        odom_points = []
#        print("inside callback")
        #for every detected car perform algorithm
        for box in boxc.bounding_boxes:
            if box.Class == "Ego":
                box_coord = [box.xmin, box.xmax, box.ymin, box.ymax]
                Xc = ((box_coord[1]-box_coord[0])/2)+box_coord[0]
                Yc = ((box_coord[3]-box_coord[2])/2)+box_coord[2]

                depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
                Z = depth_image[Yc][Xc] #reversed because cv2 array reverses width and height
                roi = depth_image[box.ymin:box.ymax, box.xmin:box.xmax]
                final_depth = np.average(roi)
#                im = plt.imshow(roi, cmap='hot')
#                plt.colorbar(im, orientation='horizontal')
#                plt.show()
#                print("average_depth" , final_depth)
#                print("Original_depth " , Z)


                camera_frame_coord = self.convert_depth_to_phys_coord_using_realsense(Xc,Yc,Z,info)

                quaternion = (
                    robo_pose.pose.orientation.x,
                    robo_pose.pose.orientation.y,
                    robo_pose.pose.orientation.z,
                    robo_pose.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)



                if((euler[2] > -3.14 and  euler[2] < -1.57-(1.57/2) ) or (euler[2]< 3.14 and euler[2] > 3.14-(1.57/2)) ):
                    camera_frame_point = [camera_frame_coord[2] + 0.4,camera_frame_coord[0] + 0.1,0]
                else:
                    camera_frame_point = [camera_frame_coord[0] + 0.1 ,camera_frame_coord[2] + 0.4,0]

                world_frame_point = [robo_pose.pose.position.x  + camera_frame_point[0] , robo_pose.pose.position.y +  camera_frame_point[1],0]
                poseStamped = PoseWithCovarianceStamped()
                poseStamped.header.frame_id = "map"
                poseStamped.header.stamp = image.header.stamp
                poseStamped.pose.pose.position.x=world_frame_point[0]
                poseStamped.pose.pose.position.y=world_frame_point[1]
                poseStamped.pose.pose.position.z=world_frame_point[2]
                poseStamped.pose.covariance = [0.0012  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0102  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0001  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.001 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.001 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.001]
                self.publish_ego_world_coord.publish(poseStamped)
 


    def world_and_car_frame(self,points, stamp):
        
        #pass xyz of camera frame  and origin frame name
        camera_point = PointStamped()
        camera_point.header.frame_id = self.origin_frame
        #camera_point.header.stamp = stamp
        camera_point.point.x=points[0]
        camera_point.point.y=points[1]
        camera_point.point.z=points[2]

        #transform to world frame coordinates
        world_point = self.listener_camera_to_world_trans.transformPoint(self.target_frame_world,camera_point)
        world_point.point.z = 0.0
        

        #transform to car frame coordinates
        car_point=self.listener_camera_to_car_trans.transformPoint(self.target_frame_car,camera_point)
        car_point.point.z = 0.0

        return [world_point, car_point]

if __name__ == '__main__':
    rospy.init_node('object_location_car')

    server = Server()
    
    #syncronyze all three topics
    image_sub = message_filters.Subscriber(server.depth_image_topic, Image)
    info_sub = message_filters.Subscriber(server.depth_camera_info_topic, CameraInfo)
    box = message_filters.Subscriber(server.bounding_box_topic, BoundingBoxes)
    robot_pose = message_filters.Subscriber(server.robot_position_topic, PoseStamped)
    ts = message_filters.TimeSynchronizer([image_sub, info_sub, box, robot_pose], 100)
    ts.registerCallback(server.callback)

    rospy.spin()
