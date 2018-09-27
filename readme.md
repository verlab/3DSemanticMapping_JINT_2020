# Usage

## Configuration

Install ROS Kinetic, turtlebot packages, CUDA. 

## Configure ROS workspace (if not set already): 

>source /opt/ros/kinetic/setup.bash
>mkdir -p ~/catkin_ws/src
>cd ~/catkin_ws/
>catkin_make
>echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc

## Clone repository to src/ folder

>cd ~/catkin_ws/src
>git clone --recurse-submodules https://www.verlab.dcc.ufmg.br/gitlab/dhiegomaga/turtlebot.git

## Build packages in order
__Check line 23 in file "~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt" to assert cuda compile version is compatible to [your graphics card version](https://developer.nvidia.com/cuda-gpus). __

>cd ~/catkin_ws/
>catkin_make --pkg driver_common
>catkin_make --pkg driver_base
>catkin_make --pkg darknet_ros_msgs
>catkin_make --pkg custom_msgs
>catkin_make -DCMAKE_BUILD_TYPE=Release

# Testing

## Offline experiments, using recorded robot data streams (dataset)

Download rosbag (dataset)

>a
>wget https://www.verlab.dcc.ufmg.br/hyperlapse/downloads/turtlebot_semantic_mapping/bag_dataset.zip

Inicializar nós do robô
>roslaunch auto initialize.launch

Executar replay
>./slam-replay.sh


**PARA EVITAR ERROS**
1. Se estiver usando a camera astra, editar a flag 'publish_tf' para 'false', no arquivo astra.launch

