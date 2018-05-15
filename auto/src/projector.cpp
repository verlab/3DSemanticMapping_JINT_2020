#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>

// Frame transformations
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

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


std::string node_topic = "projector";
std::string ref_frame = "camera_rgb_optical_frame";
std::string global_frame = "map";
//std::string global_frame = "camera_link";
std::string pointcloud_topic = "camera/depth/points";
std::string boxes_topic = "darknet_ros/bounding_boxes";
std::string out_topic = "objects_raw";

bool use_mean = false;

float camera_fx = 527.135883f;
float camera_fy = 527.76315129f;
float camera_cx = 306.5405905;
float camera_cy = 222.41208797f;

class Projector
{
    public:
        Projector(ros::NodeHandle * node_handle, std::string ref_frame, std::string global_frame, std::string pointcloud_topic, std::string boxes_topic, std::string out_topic);
        custom_msgs::WorldObject process_cloud(std::string class_name, pcl::PointCloud<pcl::PointXYZ> obj_cloud, int xmin, int xmax, int ymin, int ymax);
        pcl::PointXYZ pointFromUV(float A, float B, float C, float D, float fx, float fy, float cx, float cy, float u, float v);

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
        void markArrow(pcl::PointXYZ start, pcl::PointXYZ end, std::string frame);

        // Callbacks
        void boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr);
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr);
};

Projector::Projector(ros::NodeHandle * node_handle, std::string ref_frame, std::string global_frame, std::string pointcloud_topic, std::string boxes_topic, std::string out_topic)
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

custom_msgs::WorldObject Projector::process_cloud(std::string class_name, pcl::PointCloud<pcl::PointXYZ> obj_cloud, int xmin, int xmax, int ymin, int ymax)
{
    custom_msgs::WorldObject obj;
    obj.objClass = class_name;

    // door
    if (class_name == "door")
    {
        // Apply RANSAC in ref_frame

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

        // 1. Calculates object location in ref_frame: 
        ROS_INFO_STREAM("\nInliers count: "+ std::to_string( inliers.indices.size()));
        pcl::PointXYZ obj_position;

        // Object location is the mean of points inside bounding box
        if(use_mean)
        {
            double mean_ratio = 1.0f/inliers.indices.size();
            for(int i = 0; i < inliers.indices.size(); i++)
            {
                int index = inliers.indices.at(i);
                obj_position.x += obj_cloud.at(index).x*mean_ratio;
                obj_position.y += obj_cloud.at(index).y*mean_ratio;
                obj_position.z += obj_cloud.at(index).z*mean_ratio;
            }
        }

        // Object location is the middle point between two projections of bounding boxes into the plane found
        if(!use_mean)
        {
            float u1 = (float) xmin;
            float v1 = (float) ymin;

            float u2 = (float) xmax;
            float v2 = (float) ymax;

            float v_mean =(v1+v2)*0.5f;

            // Left projection
            pcl::PointXYZ p1 = pointFromUV(coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3], camera_fx, camera_fy, camera_cx, camera_cy, u1, v_mean);
            pcl::PointXYZ p2 = pointFromUV(coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3], camera_fx, camera_fy, camera_cx, camera_cy, u2, v_mean);
            pcl::PointXYZ p_middle;
            p_middle.x = (p1.x+p2.x)/2.0f;
            p_middle.y = (p1.y+p2.y)/2.0f;
            p_middle.z = (p1.z+p2.z)/2.0f;

            obj_position = p_middle;
        }
        
        // Convert location and normal of object from ref_frame to global_frame
        tf::StampedTransform transform;
        try{
            ros::Time now = ros::Time::now();
            listener->waitForTransform(global_frame, ref_frame, now, ros::Duration(3.0));
            listener->lookupTransform(global_frame, ref_frame, ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("Error transforming point\n!");            
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // Rotation matrix and translation vector
        tf::Matrix3x3 rot_tf = transform.getBasis();
        tf::Vector3 trans_tf = transform.getOrigin();
        
        // optional: use eigen for matrix multiplication?
        Eigen::Vector3d obj_pos_eigen(obj_position.x, obj_position.y, obj_position.z);
        Eigen::Matrix3d Rot;
        Eigen::Vector3d Trans;
        tf::matrixTFToEigen(rot_tf, Rot);
        tf::vectorTFToEigen(trans_tf, Trans);

        //ROS_INFO_STREAM("\nRot matrix " << Rot);

        // Rotate and translate position
        obj_pos_eigen = Rot * obj_pos_eigen;
        obj_pos_eigen = obj_pos_eigen + Trans;
        
        // Set position back from eigen
        obj_position.x = obj_pos_eigen(0);
        obj_position.y = obj_pos_eigen(1);
        obj_position.z = obj_pos_eigen(2);

        // Apply rotation to normal
        Eigen::Vector3d obj_normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        obj_normal = Rot * obj_normal;

        // Plot normal
        if(coefficients.values.size() == 4)
        {
            pcl::PointXYZ start, end;
            start = obj_position;
            end = obj_position;
            end.x += obj_normal(0) * 0.8;
            end.y += obj_normal(1) * 0.8;
            end.z += obj_normal(2) * 0.8;
            markArrow(start, end, global_frame);
        }

        // Calculates object angle in ref_frame (in XY plane):
        // It is the angle of -normal in XY plane (Z is upwards in usual map frame)
        float x = obj_normal(0);
        float y = obj_normal(1);
        double angle = atan2(y, x);
        obj.x = obj_position.x;
        obj.y = obj_position.y;
        obj.angle = angle;
    }

    else
    {
        ROS_INFO_STREAM("\nUnimplemented");
    }

    return obj;
}

// Projects the (u,v) image point into the plane and find 3D point
pcl::PointXYZ Projector::pointFromUV(float A, float B, float C, float D, float fx, float fy, float cx, float cy, float u, float v)
{
    pcl::PointXYZ p;

    p.z = -(D*fx*fy)/(fy*A*(u-cx) + fx*B*(v-cy) + C*fx*fy);
    p.x = (u - cx)*p.z/fx;
    p.y = (v - cy)*p.z/fy;

    return p;
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
        object = process_cloud(class_name, cropped, xmin, xmax, ymin, ymax);

        // Publish encountered object
        obj_pub.publish(object);
    }
}

void Projector::cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr)
{
    // Transform pointcloud frame 
    sensor_msgs::PointCloud2 cloud2;
    try{
        ros::Time now = ros::Time::now();
        listener->waitForTransform("camera_rgb_optical_frame", ref_frame, now, ros::Duration(10.0) );
        //listener->waitForTransform("camera_rgb_optical_frame", ref_frame, ros::Time(0), ros::Duration(10.0) );
        pcl_ros::transformPointCloud(ref_frame, *cloud2_ptr, cloud2, *listener);

    } catch (tf::TransformException ex) {
        ROS_ERROR("Error transforming cloud\n!");        
        ROS_ERROR("%s",ex.what());
    }

    // Transform pointcloud type and save to buffer
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg( cloud2, cloud);
    cloud_buffer = pcl::PointCloud<pcl::PointXYZ>(cloud);
    //pcl::fromROSMsg( cloud2, cloud_buffer);
}

void Projector::markArrow(pcl::PointXYZ start, pcl::PointXYZ end, std::string frame)
{
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> points(2);
    points[0].x = start.x;
    points[0].y = start.y;
    points[0].z = start.z;
    points[1].x = end.x;
    points[1].y = end.y;
    points[1].z = end.z;

    marker.header.frame_id = frame;
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

int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, node_topic);
    ros::NodeHandle * nh = new ros::NodeHandle;

    Projector proj(nh, ref_frame, global_frame, pointcloud_topic, boxes_topic, out_topic);
    
    // Spin
    ros::spin ();
}