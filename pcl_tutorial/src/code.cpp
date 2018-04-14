#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/ros/conversions.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>

tf::TransformListener * listener;
ros::Publisher pub;
ros::Publisher vis_pub;

/** 
 * Annoying conversions table
 * 
 * 
 * ///////////////////////////////////
 * 
 * pcl::PCLPointCloud2 <-> pcl::PointCloud<pcl::PointXYZ>

   pcl::PCLPointCloud2 point_cloud2;
   pcl::PointCloud<pcl::PointXYZ> point_cloud;

   pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
   pcl::toPCLPointCloud2(point_cloud, point_cloud2);

* /////////////////////////////////////
*
* sensor_msgs::PointCloud2 <-> pcl::PointCloud<pcl::PointXYZ>
* 
   sensor_msgs::PointCloud2 msg_cloud2;
   pcl::PointCloud<pcl::PointXYZ> point_cloud;

   pcl::fromROSMsg(msg_cloud2, point_cloud);
   pcl::toROSMsg(point_cloud, msg_cloud2);
*
**/

void publish_marker(float x, float y, float z, float x2, float y2, float z2)
{
    std::vector<geometry_msgs::Point> points(2); 
    points[0].x = x;
    points[0].y = y;
    points[0].z = z;
    points[1].x = x2;
    points[1].y = y2;
    points[1].z = z2;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.ns = "planes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    /*marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;*/
    marker.scale.x = 0.02;
    marker.scale.y = 0.05;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points = points;
    vis_pub.publish( marker );
}

// Using sensor_msgs::PCLPointCloud2
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud2_ptr)
{
    // Transform pointcloud frame 
    sensor_msgs::PointCloud2 cloud2;

    try{
        //listener.waitForTransform("", "", ros::Time(0), ros::Duration(10.0) );
        pcl_ros::transformPointCloud("camera_link", *cloud2_ptr, cloud2, *listener);

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    // Transform pointcloud type
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg( cloud2, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // Convert the thing

    seg.setInputCloud (cloud.makeShared());
    seg.segment (inliers, coefficients); 

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);

    if(coefficients.values.size() == 4)
    {
        float x, y, z, x2, y2, z2;
        x = -coefficients.values[0] * coefficients.values[3];
        y = -coefficients.values[1] * coefficients.values[3];
        z = -coefficients.values[2] * coefficients.values[3];
        x2 = x *1.4f;
        y2 = y *1.4f;
        z2 = z *1.4f;
        publish_marker(x, y, z, x2, y2, z2);
    }

    ROS_INFO_STREAM("\nsome number " << 45);
}

// Using pcl::PCLPointCloud2
/*
void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud2_ptr)
{
    


    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::fromPCLPointCloud2( *cloud2_ptr, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // Convert the thing

    seg.setInputCloud (cloud.makeShared());
    seg.segment (inliers, coefficients); 

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);

    std::string s = "what";
    ROS_INFO_STREAM("\nsome " << 45);
}
*/

int
main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "pcl_tutorial");
    ros::NodeHandle nh;

    listener = new(tf::TransformListener);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    // Spin
    ros::spin ();
}