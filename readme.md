# Installation

### Environment

Install ROS Kinetic, turtlebot packages, CUDA. 

### Configure ROS workspace (if not set already): 

>source /opt/ros/kinetic/setup.bash

>mkdir -p ~/catkin_ws/src

>cd ~/catkin_ws/

>catkin_make

>echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

### Clone repository to src/ folder

>cd ~/catkin_ws/src

>git clone --recurse-submodules https://www.verlab.dcc.ufmg.br/gitlab/dhiegomaga/turtlebot.git

### Build packages in order
*Check line 23 in file "~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt" to assert cuda compile version is compatible to [your graphics card version](https://developer.nvidia.com/cuda-gpus).*

>cd ~/catkin_ws/

>catkin_make --pkg driver_common

>catkin_make --pkg driver_base

>catkin_make --pkg darknet_ros_msgs

>catkin_make --pkg custom_msgs

>catkin_make -DCMAKE_BUILD_TYPE=Release

# Testing

### Offline test, using recorded robot data streams (dataset)

Download rosbag (dataset):

>mkdir _/path/to/dataset/folder_

>cd _/path/to/dataset/folder_

>wget https://www.verlab.dcc.ufmg.br/hyperlapse/downloads/turtlebot_semantic_mapping/bag_dataset.zip

Play rosbag:

>cd dataset

>./slam_replay

initialize yolo_detector nodes... 

### Online test, using physical robot

_Requirements: Kobuki base/other turtlebot base, RGBD camera, laser scan (optional) (RGBD camera depth stream can be converted to laser scan, but usually has lower range and accuracy)._

Start base: 

>roslaunch auto initialize.launch

initialize slam and yolo_detector nodes... 

**Notes**
*Before usage, check that no packages publish tf transformations, i.e., 'publish_tf' flag in launch files are set to* __false.__ *Only the rosbag play and robot description launch files should publish tf's.*

### Offline Run Updated
Note: catkin_ws is the PATH to the folder where you compiled the turtlebot project
* (terminal 1) roscore
* (terminal 2) cd catkin_ws/src/turtlebot/auto/launch; roslaunch yolo_detector.launch
* (terminal 3) cd catkin_ws/src/turtlebot/auto/launch; roslaunch auto obj_positioner.launch
* (terminal 4) cd PATH_TO_DATASET; ./slam-replay
* (terminal 5) rviz // and then use the rviz GUI to open rviz config file "rviz_config.rviz" in catkin_ws/src/turtlebot for the vizualization
