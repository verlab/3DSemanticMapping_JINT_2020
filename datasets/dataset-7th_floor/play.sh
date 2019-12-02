rosparam set /use_sim_time true; roslaunch openni2_launch openni2.launch load_driver:=false & sleep 3; rosbag play -r 1.0 --clock files/7th_floor.bag & roslaunch files/robot_desc.launch
