## 1 - Install ROS Kinetic with required ROS packages (e.g., openni, RTAB-Map)

We tested the program with ROS Kinetic. The full installation steps are given in http://wiki.ros.org/kinetic/Installation/Ubuntu
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
source ~/.bashrc
```
Sanity check: *roscd* command should work now to any of available packages such as "perception":
```shell
roscd perception 
cd 
```
Please install openni, turtlebot and RGB-D SLAM packages:
```shell
sudo apt install ros-kinetic-openni*
sudo apt install ros-kinetic-turtlebot
sudo apt install ros-kinetic-rtabmap-ros
source ~/.bashrc 
```
## 2 - Install and configure Catkin build tools

Now we need to install and configure catkin package. We prefered the more recent *catkin-tools* package than *catkin_make*.
Please also have a look at [catkin-tools package](https://catkin-tools.readthedocs.io/en/latest/installing.html) for more details and further commands. 
```shell
sudo apt-get install python-catkin-tools
source ~/.bashrc 
```
And then set a new catkin workspace. *IMPORTANT NOTE:* you can choose any location to set your workspace. Here we chose the to place the workspace folder at *~/code/catkin_ws*. To set another location, just change *~/code/catkin_ws* to your preferred location.
```shell
mkdir -p ~/code/catkin_ws/src
cd ~/code/catkin_ws
catkin init
echo "source ~/code/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
A warning message on your terminal might pop-up if you dont have any package in the catkin workspace yet. This message will disapear as soon as you build a project in your workspace later. 

