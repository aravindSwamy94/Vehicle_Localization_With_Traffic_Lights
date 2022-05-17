#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <test> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from morse.sensors import *
from fourwd.builder.robots import Hummerscaled
from fourwd.builder.robots.pole import Pole
from fourwd.builder.robots.dummy_car import DummyCar
from fourwd.builder.robots.dummy_car_2 import DummyCar2
from fourwd.builder.actuators.dummy_car_keyboard import DummyKeyboard
from fourwd.builder.actuators.slim_keyboard import SlimKeyboard

import math
import logging
import sys

logger = logging.getLogger("morse." + __name__)

# CONSTANTS for DUMMY CARS
dummy_start_pos = []

# dummy car position
Dummy_X = -9
Dummy_Y = 0.5
dummy_start_pos.append((Dummy_X, Dummy_Y))

# HUMMER ROBOT
robot = Hummerscaled()
robot.add_default_interface('ros')
scale = 0.12
robot.properties(scale=scale)
robot.properties(GroundRobot=True)
robot.properties(RobotFrame=True)
robot.name = "FourWD"
robot.scale = [scale, scale, scale]

# This is the wheel odometry tf
odom = Odometry()
odom.add_stream('ros', frame_id="odom", topic="wheel_odom", child_frame_id='wheel_odom')  # child_frame_id='base_link')
odom.alter('Noise', pos_std=0.08, rot_std=math.radians(2))
odom.translate(0.0, 0.0, 0.0)
odom.rotate(0.0, 0.0, 0)

# IMU sensor located inside the camera
imu = IMU()
imu.name = "imu"
imu.add_stream('ros', frame_id='camera_imu_optical_frame', topic='imu/data')
imu.alter('Noise', pos_std=0.08, rot_std=math.radians(2))
imu.translate(0.6, 0.0, 1.2)
imu.rotate(0.0, 0.0, 0.0)

# Add a pose sensor that exports the current location and orientation. Note that this is only for testing purposes
pose = Pose()
pose.add_stream('ros', frame_id="map", topic='pose')


# creates a gps sensor
gps = GPS()
gps.translate(0, 0, 0)
gps.rotate(0, 0, 0)
gps.alter('Noise', pos_std=0.08, rot_std=math.radians(2))

gps.add_stream('ros', frame_id="map", topic='/gps/data')


# The list of the maikinect = Kinect() kinect.depth_camera.add_stream('ros', frame_id="camera_depth_frame", topic='/camera/depth', parent_frame_id="camera_link", topic_suffix='/image_raw') kinect.depth_camera.properties(cam_width=256, cam_height=256, cam_near = 0.2, cam_far = 10.0) kinect.video_camera.add_stream('ros', frame_id="camera_color_frame", topic='/camera/rgb', parent_frame_id="camera_link", topic_suffix='/image_raw') kinect.video_camera.properties(cam_width=256, cam_height=256) kinect.translate(0.6, -0.1, 1.2) kinect.rotate(0.0, 0.0, 0)n methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/astable/user/builder_overview.html
robot.translate(-5.33073, -7.0, 0.1)
robot.rotate(0.0, 0.0, math.pi / 2)
robot.set_mass(1.5)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators

# Twist_Force_Actuator is an controller designed to use appropriate force according to desired velocity
actuator = SlimKeyboard()

robot.append(imu)
robot.append(actuator)
robot.append(odom)
robot.append(pose)
robot.append(gps)

# a basic keyboard controller for testing purposes
keyboard = SlimKeyboard()
robot.append(keyboard)



# DUMMY CAR
dummy = DummyCar(name="DummyCar")
dummy.add_default_interface('ros')
scale = 0.15
dummy.properties(RobotFrame=True)
dummy.scale = [scale, scale, scale]
dummy.translate(Dummy_X, Dummy_Y, 0.1)
dummy.rotate(0.0, 0.0, 0.0)
dummy.set_mass(1)
dummy_pos = Pose()

dummy_pos.add_stream('ros', frame_id="dummy_car", topic='DummyCar/pose')

dummy_keyboard = DummyKeyboard()
motion = MotionVW()
dummy_keyboard.properties(Id=0)
dummy.append(dummy_keyboard)
dummy.append(dummy_pos)
dummy.append(motion)

# DUMMY CAR_2

dummy2 = DummyCar2(name="DummyCar2")
dummy2.add_default_interface('ros')
scale = 0.18
dummy2.properties(RobotFrame=True)
dummy2.scale = [scale, scale, scale]
dummy2.translate(-5.33073, -10.0, 0.1)
dummy2.rotate(0.0, 0.0, math.pi / 2)
dummy2.set_mass(1)
dummy2_pos = Pose()

dummy2_pos.add_stream('ros', frame_id="dummy_car_2", topic='DummyCar_2/pose')

kinect = Kinect(name = 'dummy2_car')
kinect.depth_camera.add_stream('ros', frame_id="camera_depth_frame", topic='/camera/depth', parent_frame_id="dummy_car_2", topic_suffix='/image_raw')
kinect.depth_camera.properties(cam_width=256, cam_height=256)
kinect.video_camera.add_stream('ros', frame_id="camera_color_frame", topic='/camera/rgb', parent_frame_id="dummy_car_2", topic_suffix='/image_raw')
kinect.video_camera.properties(cam_width=256, cam_height=256)
kinect.translate(0.0, 0.0, 2.5)
kinect.rotate(0.0, 0.0, 3.14)


dummy2_keyboard = DummyKeyboard()
motion2 = MotionVW()
dummy2_keyboard.properties(Id=1)
dummy2.append(dummy2_keyboard)
dummy2.append(dummy2_pos)
dummy2.append(motion2)
dummy2.append(kinect)

# INFRASTRUCTURE CAMERA
roboPole = Pole()
roboPole.add_default_interface('ros')
roboPole.properties(GroundRobot=True)
# Infrastructure camera properties
infrastructure_camera = Kinect(name='infrastructure_camera')
infrastructure_camera.depth_camera.add_stream('ros', frame_id="infrastructure_camera_depth_frame",
                                              topic='/infrastructure/camera/depth', parent_frame_id="infra_camera_link",
                                              topic_suffix='/image_raw')
# original
infrastructure_camera.depth_camera.properties(cam_width=256, cam_height=256)
# infrastructure_camera.depth_camera.properties(cam_width=768, cam_height=768)
infrastructure_camera.add_stream('ros', frame_id="infrastructure_camera_color_frame",
                                 topic='/infrastructure/camera/rgb', parent_frame_id="infra_camera_link",
                                 topic_suffix='/image_raw')

# original
infrastructure_camera.video_camera.properties(cam_width=256, cam_height=256)
# infrastructure_camera.video_camera.properties(cam_width=768, cam_height=768)
#infrastructure_camera.translate(0.1, 0.0, 0.1)
# TO ADJUST THE CAMERA VIEW, CHANGE THE ORIENTAION BELOW: 2nd variable is PITCH (around y axis) and 3rd variable is YAW (around Z axis)

# from behind the wall
infrastructure_camera.rotate(0, math.pi / 5, -math.pi / 1.2)

# positioning of the robot pole
#roboPole.translate(-3.00, 1.5, 0.1)  # CHANGE THE POSITION (X,Y) OF THE POLE IN WORLD COORDIANTES TO MOVE THE CAMERA IN THE WORLD
roboPole.translate(-3.00, 1.5, 0.1)  # CHANGE THE POSITION (X,Y) OF THE POLE IN WORLD COORDIANTES TO MOVE THE CAMERA IN THE WORLD
roboPole.rotate(0.0, 0.0, 0.0)
roboPole.append(infrastructure_camera)


# set 'fastmode' to True to switch to wireframe mode
env = Environment('fourwd/environments/aras_4_two_cars_final.blend', fastmode=False)

env.set_camera_location([5, 1, 15])
env.set_camera_rotation([math.radians(35), 0, math.radians(90)])
env.properties(latitude=0, longitude=45.1, altitude=0.0)
env.set_viewport(viewport_shade='TEXTURED', clip_end=1000)
env.show_framerate(True)
env.add_stream('ros')
