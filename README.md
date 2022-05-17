###Software and Hardware Requirements
- Ubuntu 18.04
- Nvidia Graphics (min GTX 970, 4GB memory)
- Cuda (min 11.0)
- ROS (melodic)


### Installation - Pre requisites
```
The project runs on ROS melodic and the simulation env is Morse 1.4_STABLE versions.
First, install ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu 

Then install Morse from source, (ref: http://www.openrobots.org/morse/doc/1.3/user/installation/mw/ros.html)

Install Nvidia and cuda driver for ubuntu 
https://gist.github.com/wangruohui/df039f0dc434d6486f5d4d098aa52d07
[please follow the instructions in above link to install nvidia and cuda driver]

Robot Localization package - ros:
sudo apt-get install ros-melodic-robot-localization

```
### Build and Run Simulation

#### Build darknet_ros ,darknet and framework
```
cd src/

git clone https://github.com/tom13133/darknet_ros.git

cd darknet_ros/

rm -rf darknet/

git clone https://github.com/AlexeyAB/darknet.git

cd ../../

bash copy_files.sh

catkin_make

```

#### Run Simulation
```
cd morse

morse import fourwd

morse run fourwd fourwd/simulation.py
```

#### Run EKF

```
source devel/setup.bash

roslaunch pose_publisher pose_publisher.launch

```

#### Run Infrastructure based detection

```
source devel/setup.bash

roslaunch darknet_ros yolo_v4.launch

roslaunch object_location camera_to_world.launch 



```
#### Run MATLAB GUI

```
cd src/MATLAB-GUI/
Open matlab and Run app1.mlapp

```
