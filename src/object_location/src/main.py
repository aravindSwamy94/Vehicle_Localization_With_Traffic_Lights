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

class Server:
    def __init__(self):
        self.bridge = CvBridge()
        self.robot_car_position_threshhold = 0.6
        self.robot_position_topic = '/pose'
        self.depth_image_topic = '/roboPole/infrastructure_camera/depth/image'
        self.depth_camera_info_topic = '/roboPole/infrastructure_camera/depth/camera_info'
        self.bounding_box_topic = '/darknet_ros/bounding_boxes'
        self.topic_name_world = "three_dim_point_world"
        self.topic_name_car = "three_dim_point_car"
        self.target_frame_world = "/map"
        self.target_frame_car = "/odom"
        self.origin_frame = "infrastructure_camera_depth_frame"
        self.marker_topic = "obstacle_marker_visualization"
        self.listener_camera_to_world_trans = tf.TransformListener()
        self.listener_camera_to_car_trans= tf.TransformListener()
        self.publish_world_coord = rospy.Publisher(self.topic_name_world, PointStamped, queue_size=10)
        self.publish_car_coord = rospy.Publisher(self.topic_name_car, PointStamped, queue_size=10)
        self.publish_marker = rospy.Publisher(self.marker_topic,Marker,queue_size=10)
        self.publish_ego_world_coord = rospy.Publisher("/pose_traffic", PoseWithCovarianceStamped, queue_size=10)
        try:
            self.listener_camera_to_world_trans.waitForTransform(self.origin_frame, self.target_frame_world, rospy.Time(0),rospy.Duration(10.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            print(err)
        try:
            self.listener_camera_to_car_trans.waitForTransform(self.origin_frame, self.target_frame_car, rospy.Time(0),rospy.Duration(10.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            print(err)
        



    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth, cameraInfo):
        Xreal = ((x-cameraInfo.K[2])*depth)/cameraInfo.K[0]
        Yreal = ((y-cameraInfo.K[5])*depth)/cameraInfo.K[4]
        result = [Xreal, Yreal, depth]
        return result



    def callback(self, image, info, boxc, robo_pose):
        world_points = []
        odom_points = []

        #for every detected car perform algorithm
        for box in boxc.bounding_boxes:
            if box.Class == "Ego":
                box_coord = [box.xmin, box.xmax, box.ymin, box.ymax]
                Xc = ((box_coord[1]-box_coord[0])/2)+box_coord[0]
                Yc = ((box_coord[3]-box_coord[2])/2)+box_coord[2]

                depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
                Z = depth_image[Yc][Xc] #reversed because cv2 array reverses width and height
                
                #get camera frame xyz           
                camera_frame_coord = self.convert_depth_to_phys_coord_using_realsense(Xc,Yc,Z,info)

                #get other frames xyz
                world_car_points = self.world_and_car_frame(camera_frame_coord, image.header.stamp)

                poseStamped = PoseWithCovarianceStamped()
                poseStamped.header.frame_id = "map"
                poseStamped.header.stamp = image.header.stamp
                poseStamped.pose.pose.position.x=world_car_points[0].point.x - 0.03
                poseStamped.pose.pose.position.y=world_car_points[0].point.y - 0.17
                poseStamped.pose.pose.position.z=world_car_points[0].point.z 
                poseStamped.pose.covariance = [0.0078  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0034  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0001  ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.0000 ,0.000]
                self.publish_ego_world_coord.publish(poseStamped)

            if box.Class == "Obs":
                #extract middle point depth value of bounding box 
                box_coord = [box.xmin, box.xmax, box.ymin, box.ymax]
                Xc = ((box_coord[1]-box_coord[0])/2)+box_coord[0]
                Yc = ((box_coord[3]-box_coord[2])/2)+box_coord[2]

                depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
                Z = depth_image[Yc][Xc] #reversed because cv2 array reverses width and height
                
                #get camera frame xyz           
                camera_frame_coord = self.convert_depth_to_phys_coord_using_realsense(Xc,Yc,Z,info)

                #get other frames xyz
                world_car_points = self.world_and_car_frame(camera_frame_coord, image.header.stamp)
                #check to see if robot car detected
                if abs(world_car_points[0].point.x - robo_pose.pose.position.x) < self.robot_car_position_threshhold and abs(world_car_points[0].point.y - robo_pose.pose.position.y) < self.robot_car_position_threshhold:
                    continue

#                world_points.append(world_car_points[0])
#                odom_points.append(world_car_points[1])
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = 0.4
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0  
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = world_car_points[0].point.x
                marker.pose.position.y =  world_car_points[0].point.y
                marker.pose.position.z =  world_car_points[0].point.z  
                self.publish_marker.publish(marker)
                self.publish_world_coord.publish(world_car_points[0])
                self.publish_car_coord.publish(world_car_points[1])



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
    rospy.init_node('object_location')

    server = Server()
    
    #syncronyze all three topics
    image_sub = message_filters.Subscriber(server.depth_image_topic, Image)
    info_sub = message_filters.Subscriber(server.depth_camera_info_topic, CameraInfo)
    box = message_filters.Subscriber(server.bounding_box_topic, BoundingBoxes)
    robot_pose = message_filters.Subscriber(server.robot_position_topic, PoseStamped)
    ts = message_filters.TimeSynchronizer([image_sub, info_sub, box, robot_pose], 100)
    ts.registerCallback(server.callback)

    rospy.spin()
