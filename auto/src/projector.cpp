#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>

// WorldObject
#include "custom_msgs/WorldObject.h"

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

std::string node_topic = "projector";
std::string ref_frame = "camera_link";
std::string pointcloud_topic = "camera/depth/points";
std::string boxes_topic = "darknet_ros/bounding_boxes";
std::string out_topic = "objects";

class Projector
{
    public:
        Projector(ros::NodeHandle * node_handle, std::string node_topic, std::string ref_frame, std::string pointcloud_topic, std::string boxes_topic, std::string out_topic);

        custom_msgs::WorldObject process_cloud(std::string class_name, pcl::PointCloud<pcl::PointXYZ> obj_cloud);

    private:

        tf::TransformListener * listener;
        ros::NodeHandle * nh;
        pcl::PointCloud<pcl::PointXYZ> cloud_buffer;

        // Subscribers
        ros::Subscriber cloud_sub;
        ros::Subscriber boxes_sub; 

        // Publishers
        ros::Publisher obj_pub;

        // Debug variables
        ros::Publisher vis_pub;
        void markArrow(pcl::PointXYZ start, pcl::PointXYZ end);

        // Callbacks
        void boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr);
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr);
};

Projector::Projector(ros::NodeHandle * node_handle, std::string node_topic, std::string ref_frame, std::string pointcloud_topic, std::string boxes_topic, std::string out_topic)
{
    nh = node_handle;
    listener = new tf::TransformListener;

    // Initialize Subscribers
    cloud_sub = nh->subscribe(pointcloud_topic, 1, &Projector::cloud_callback, this);
    boxes_sub = nh->subscribe(boxes_topic, 1, &Projector::boxes_callback, this);

    // Initialize Publishers
    obj_pub = nh->advertise<custom_msgs::WorldObject>(out_topic, 1);

    // Initialize Debug variables
    vis_pub = nh->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
}

custom_msgs::WorldObject Projector::process_cloud(std::string class_name, pcl::PointCloud<pcl::PointXYZ> obj_cloud)
{
    custom_msgs::WorldObject obj;
    obj.objClass = class_name;

    // door
    if (class_name == "door")
    {   
        // APPLY RANSAC

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
        seg.setInputCloud (obj_cloud.makeShared());
        seg.segment (inliers, coefficients); 

        // Mean point location: 
        ROS_INFO_STREAM("\nInliers count: "+ std::to_string( inliers.indices.size()));
        pcl::PointXYZ mean;
        double mean_ratio = 1.0f/inliers.indices.size();
        for(int i = 0; i < inliers.indices.size(); i++)
        {
            int index = inliers.indices.at(i);
            mean.x += obj_cloud.at(index).x*mean_ratio;
            mean.y += obj_cloud.at(index).y*mean_ratio;
            mean.z += obj_cloud.at(index).z*mean_ratio;
        }
        
        // Plot normal
        if(coefficients.values.size() == 4)
        {
            pcl::PointXYZ start, end;
            start = mean;
            end = mean;
            end.x -= coefficients.values[0] * 0.8;
            end.y -= coefficients.values[1] * 0.8;
            end.z -= coefficients.values[2] * 0.8;
            markArrow(start, end);
        }
    }

    else
    {
        ROS_INFO_STREAM("\nUnimplemented");
    }

    return obj;
}

void Projector::boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr)
{
    int box_num = boxes_ptr->bounding_boxes.size();

    for(int i = 0; i < box_num; i++)
    {
        // Object class
        std::string class_name = boxes_ptr->bounding_boxes.at(i).Class;
        custom_msgs::WorldObject object; 

        // Object boundaries
        int xmin = boxes_ptr->bounding_boxes.at(i).xmin;
        int ymin = boxes_ptr->bounding_boxes.at(i).ymin;
        int xmax = boxes_ptr->bounding_boxes.at(i).xmax;
        int ymax = boxes_ptr->bounding_boxes.at(i).ymax;

        // Crop pointcloud inside bounding box
        pcl::PointCloud<pcl::PointXYZ> cropped; 
        cropped.width = std::abs(xmax-xmin);
        cropped.height = std::abs(ymax-ymin);
        cropped.points.resize (cropped.width * cropped.height);

        //ROS_INFO_STREAM("\nPointcloud dim: "+ std::to_string(cloud_buffer.width) + " "+ std::to_string(cloud_buffer.height));

        for(int x = 0; x < cropped.width; x++)
            for(int y = 0; y < cropped.height; y++)
                cropped.at(x, y) = cloud_buffer.at(x+xmin, y+ymin);

        // Set frame
        //cropped.header.frame_id = ref_frame;

        /**
        // Convert to ros msg
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cropped, cloud_msg);

        // Set frame 
        cloud_msg.header.frame_id = ref_frame;
        **/
        // Process data in core function to generate a world object
        object = process_cloud(class_name, cropped);
    }
}

void Projector::cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr)
{
    // Transform pointcloud frame 
    sensor_msgs::PointCloud2 cloud2;
    try{
        //listener.waitForTransform("", "", ros::Time(0), ros::Duration(10.0) );
        pcl_ros::transformPointCloud(ref_frame, *cloud2_ptr, cloud2, *listener);

    } catch (tf::TransformException ex) {
        ROS_INFO_STREAM("killme\n");
        ROS_ERROR("%s",ex.what());
    }

    // Transform pointcloud type and save to buffer
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg( cloud2, cloud);
    cloud_buffer = pcl::PointCloud<pcl::PointXYZ>(cloud);
    //pcl::fromROSMsg( cloud2, cloud_buffer);
}

void Projector::markArrow(pcl::PointXYZ start, pcl::PointXYZ end)
{
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> points(2);
    points[0].x = start.x;
    points[0].y = start.y;
    points[0].z = start.z;
    points[1].x = end.x;
    points[1].y = end.y;
    points[1].z = end.z;

    marker.header.frame_id = ref_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "arrows";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
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

void boxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr)
{
    // For each found element
    int a = boxes_ptr->bounding_boxes.size();
    for(int i = 0; i < a; i++)
    {
        // Object class
        std::string str = boxes_ptr->bounding_boxes.at(i).Class;

        // Object boundaries
        int xmin = boxes_ptr->bounding_boxes.at(i).xmin;
        int ymin = boxes_ptr->bounding_boxes.at(i).ymin;
        int xmax = boxes_ptr->bounding_boxes.at(i).xmax;
        int ymax = boxes_ptr->bounding_boxes.at(i).ymax;

        // Crop pointcloud inside bounding box
        pcl::PointCloud<pcl::PointXYZ> cropped; 
        cropped.width = std::abs(xmax-xmin);
        cropped.height = std::abs(ymax-ymin);
        cropped.points.resize (cropped.width * cropped.height);

        ROS_INFO_STREAM("xmin=" + std::to_string(xmin) + " xmax="+std::to_string(xmax)+ " ymin=" + std::to_string(ymin) + " ymax="+std::to_string(ymax)+"\n");
        ROS_INFO_STREAM("cropped width=" + std::to_string(cropped.width) + " cropped height="+ std::to_string(cropped.height)+ "\n");


        // Convert to ros msg
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cropped, cloud_msg);

        // Set frame 
        cloud_msg.header.frame_id = "camera_link";      
    }
}


// Using sensor_msgs::PCLPointCloud2
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr)
{
    //ROS_INFO_STREAM("\nsome number " << 45);
    sensor_msgs::PointCloud2 cloud_clone;

    // Transform pointcloud frame 
    sensor_msgs::PointCloud2 cloud2;
    

    // Transform pointcloud type
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg( cloud2, cloud);

    //cloud_buffer = pcl::PointCloud<pcl::PointXYZ>(cloud);

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
    //pub.publish (ros_coefficients);

    if(coefficients.values.size() == 4)
    {
        float x, y, z, x2, y2, z2;
        x = -coefficients.values[0] * coefficients.values[3];
        y = -coefficients.values[1] * coefficients.values[3];
        z = -coefficients.values[2] * coefficients.values[3];
        x2 = x *1.4f;
        y2 = y *1.4f;
        z2 = z *1.4f;
        //publish_marker(x, y, z, x2, y2, z2);
    }
   
}

int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, node_topic);
    ros::NodeHandle * nh = new ros::NodeHandle;

    Projector proj(nh, node_topic, ref_frame, pointcloud_topic, boxes_topic, out_topic);
    
    // Spin
    ros::spin ();
}