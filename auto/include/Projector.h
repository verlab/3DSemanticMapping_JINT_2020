#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>

// Frame transformations
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

// WorldObject
#include "custom_msgs/WorldObject.h"
#include "custom_msgs/ObjectList.h"

// Darknet
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#define point_type pcl::PointXYZRGB

using std::string; 

class Projector
{
    public:
        Projector(ros::NodeHandle * node_handle, string pointcloud_topic, string boxes_topic, string odom_topic, string detection_flag_topic, string out_topic);
        custom_msgs::WorldObject process_cloud(std::string class_name, pcl::PointCloud<point_type> obj_cloud, int xmin, int xmax, int ymin, int ymax);
        pcl::PointXYZ pointFromUV(float A, float B, float C, float D, float fx, float fy, float cx, float cy, float u, float v);
        void execute();
        /**
         * Configuration Params
         * */

        // Frames 
        string camera_frame; 
        string robot_frame;
        string global_frame; 

        // Camera intrinsics
        float camera_fx;
        float camera_fy;
        float camera_cx;
        float camera_cy;

        // Projector max distance
        float max_proj_dist;

        // If set, does not allow projections when robot is rotating
        bool rotation_optmization;

        // If set, does not use camera projection to find object position; instead, it uses the centroid of the associated pointcloud
        bool use_mean; 
        bool showClassName; 

    private:

        tf::TransformListener * listener;
        ros::NodeHandle * nh;
        pcl::PointCloud<point_type> cloud_buffer;
        tf::StampedTransform transform, transform_buffer, robot_transform;
        bool block_projection; 
        float last_rotation;
        int block_count;
        bool too_far;
        bool quiet_mode; 
        int x_offset;

        // Method to be used for object projection
        int method_door = 1; // 0: naive, 1: plane projection
        int method_bench = 2; // 0: naive, 1: plane removal + custering, 2: simple clustering
        int method_fire = 2; // 0: naive, 1: plane removal + custering, 2: simple clustering
        int method_trash = 2; // 0: naive, 1: plane removal + custering, 2: simple clustering
        int method_water = 2; // 0: naive, 1: plane removal + custering, 2: simple clustering

        // Naive appoach, window width:
        int window_width = 40; 

        // Subscribers
        ros::Subscriber cloud_sub;
        ros::Subscriber boxes_sub; 
        ros::Subscriber detection_flag_topic_sub; 
        ros::Subscriber odom_sub; 

        // Publishers
        ros::Publisher obj_pub;
        ros::Publisher obj_list_pub;
        ros::Publisher cloud_pub;

        // Debug variables
        ros::Publisher vis_pub;
        int count; 

        // Methods 
        void markArrow(pcl::PointXYZ start, pcl::PointXYZ end, std::string frame, int id, double r, double g, double b);
        void markObject(const pcl::PointXYZ & location, const std::string & className, const std::string & frame, const std::string & shape, int id, double sx, double sy, double sz, double r, double g, double b);
        float distanceFromRobot(float x, float y);
        pcl::PointXYZ convertToMapFrame(const pcl::PointXYZ & point);

        // Callbacks
        void boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr);
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr);
        void flag_callback(const std_msgs::Int8 & flag);
        void odom_callback(const nav_msgs::Odometry & odom);
};

