## Required Setup on Ubuntu 18.04 with ROS Melodic

You might enconter the following issues when trying to run the code on Ubuntu 18.04 with ROS Melodic.

### Wrong gcc and g++ specified versions and path

The `darknet_ros` code only compiles with gcc version up to 6, so you need to have the compiler installed and specify its path. If it is installed in another path you have to update the flags for the catkin build.

During the build step, we specify the following flags for building and you should change then accordingly to your gcc and g++ install locations:

```shell
catkin config --default-devel-space --default-build-space --default-install-space --default-source-space --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-6 -DCMAKE_CXX_COMPILER=/usr/bin/g++-6
```
### ROS Melodic Turtlebot2 Packages

There is currently no **official** turtlebot2 package for ROS Melodic, so we get TF errors because our set up assumes transformations are being published from the robot model server (even though those are static transformations, hence this could be circumvented using a single static publisher).

To install turtlebot2 for ROS Melodic you can use [this repository](https://github.com/gaunthan/Turtlebot2-On-Melodic).
