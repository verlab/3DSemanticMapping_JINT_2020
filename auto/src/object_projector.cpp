#include <ros/ros.h>
#include "Projector.h"

using std::string;

int main (int argc, char** argv)
{
    // NODE 
    string node_name="object_projector";

    // Initialize ROS
    ros::init (argc, argv, node_name);
    ros::NodeHandle * nh = new ros::NodeHandle("~");

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
    double max_proj_dist;

    // Get params

    nh->param("pointcloud_topic", pointcloud_topic, string("/camera/depth_registered/points"));    
    nh->param("boxes_topic", boxes_topic, string("/darknet_ros/bounding_boxes"));  
    nh->param("detection_flag_topic", detection_flag_topic, string("/darknet_ros/flag_detection"));
    nh->param("odom_topic", odom_topic, string("/odom"));
    nh->param("out_topic", out_topic, string("/objects_raw"));
    nh->param("camera_frame", camera_frame, string("/camera_rgb_optical_frame"));
    nh->param("robot_frame", robot_frame, string("/base_link"));
    nh->param("global_frame", global_frame, string("/map"));
    
    nh->param("camera_cx", camera_cx, 306.540);
    nh->param("camera_cy", camera_cy, 222.412);
    nh->param("camera_fx", camera_fx, 527.135);
    nh->param("camera_fy", camera_fy, 527.763);

    nh->param("rotation_optmization", rotation_optmization, true);
    nh->param("use_mean", use_mean, false);
    nh->param("max_proj_distance", max_proj_dist, 7.0);

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

    ROS_INFO_STREAM("\n camera_frame: "+ camera_frame );
    ROS_INFO_STREAM("\n max_proj_dist: "+ std::to_string( max_proj_dist ));

    // Spin
    ros::spin ();
}
