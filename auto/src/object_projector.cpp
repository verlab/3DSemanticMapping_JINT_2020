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
    float fx = 527.135883f;
    float fy = 527.76315129f;
    float cx = 306.5405905;
    float cy = 222.41208797f;

    // Kinect V2
    //float camera_fx = 1074.01f/2.0f;
    //float camera_fy = 1073.9f/2.0f;
    //float camera_cx = 945.3f/2.0f;
    //float camera_cy = 537.4f/2.0f;

    float camera_cx, camera_cy, camera_fx, camera_fy; 
    bool use_mean, rotation_optmization; 
    float max_proj_dist;

    // Get params
    ros::param::param<string>("pointcloud_topic", pointcloud_topic, "camera/depth_registered/points");    
    ros::param::param<string>("boxes_topic", boxes_topic, "darknet_ros/bounding_boxes");  
    ros::param::param<string>("detection_flag_topic", detection_flag_topic, "darknet_ros/flag_detection");
    ros::param::param<string>("odom_topic", odom_topic, "odom");
    ros::param::param<string>("out_topic", out_topic, "objects_raw");
    ros::param::param<string>("camera_frame", camera_frame, "camera_rgb_optical_frame");
    ros::param::param<string>("robot_frame", robot_frame, "base_link");
    ros::param::param<string>("global_frame", global_frame, "map");
    ros::param::param("camera_cx", camera_cx, cx);
    ros::param::param("camera_cy", camera_cy, cy);
    ros::param::param("camera_fx", camera_fx, fx);
    ros::param::param("camera_fy", camera_fy, fy);
    ros::param::param("rotation_optmization", rotation_optmization, true);
    ros::param::param("use_mean", use_mean, false);
    ros::param::param("max_proj_distance", max_proj_dist, 6.0f);

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
