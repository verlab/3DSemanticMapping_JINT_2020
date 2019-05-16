#include <ros/ros.h>
#include "Projector.h"

using std::string;

int main (int argc, char** argv)
{
    // NODE 
    string node_name="object_projector";

    // Initialize ROS
    ros::init (argc, argv, node_name);
    ros::NodeHandle * nh = new ros::NodeHandle;

    // Parameter variables

    // TOPICS IN
    string pointcloud_topic, boxes_topic, detection_flag_topic, odom_topic;

    // TOPIC OUT 
    string out_topic; 

    // FRAMES
    string camera_frame, robot_frame, global_frame; 

    // OTHER 
    // Astra camera
    double camera_fx = 527.135883f;
    double camera_fy = 527.76315129f;
    double camera_cx = 306.5405905;
    double camera_cy = 222.41208797f;

    // Kinect V2
    //double camera_fx = 1074.01f/2.0f;
    //double camera_fy = 1073.9f/2.0f;
    //double camera_cx = 945.3f/2.0f;
    //double camera_cy = 537.4f/2.0f;

    bool use_mean = false, rotation_optmization = true; 
    double max_proj_dist = 6.0;

    // Get params
    ros::param::param<string>("pointcloud_topic", pointcloud_topic, "camera/depth_registered/points");    
    ros::param::param<string>("boxes_topic", boxes_topic, "darknet_ros/bounding_boxes");  
    ros::param::param<string>("detection_flag_topic", detection_flag_topic, "darknet_ros/flag_detection");
    ros::param::param<string>("odom_topic", odom_topic, "odom");
    ros::param::param<string>("out_topic", out_topic, "objects_raw");
    ros::param::param<string>("camera_frame", camera_frame, "camera_rgb_optical_frame");
    ros::param::param<string>("robot_frame", robot_frame, "base_link");
    ros::param::param<string>("global_frame", global_frame, "map");

    ros::param::get("camera_cx", camera_cx);
    ros::param::get("camera_cy", camera_cy);
    ros::param::get("camera_fx", camera_fx);
    ros::param::get("camera_fy", camera_fy);
    ros::param::get("rotation_optmization", rotation_optmization);
    ros::param::get("use_mean", use_mean);
    ros::param::get("max_proj_distance", max_proj_dist);

    // Initialize and set params
    Projector projector(nh, pointcloud_topic, boxes_topic, odom_topic, detection_flag_topic,out_topic);
    projector.camera_frame = camera_frame;
    projector.robot_frame = robot_frame;
    projector.global_frame = global_frame;
    projector.camera_cx = camera_cx; 
    projector.camera_cy = camera_cy;
    projector.camera_fx = camera_fx;
    projector.camera_fy = camera_fy;
    projector.rotation_optmization = rotation_optmization; 
    projector.use_mean = use_mean;
    projector.max_proj_dist = max_proj_dist;

    // Spin
    ros::spin ();
}
