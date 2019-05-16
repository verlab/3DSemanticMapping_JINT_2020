#include "Projector.h"

using std::string;

Projector::Projector(ros::NodeHandle * node_handle, string pointcloud_topic, string boxes_topic, string odom_topic, string detection_flag_topic, string out_topic)
{
    nh = node_handle;
    listener = new tf::TransformListener;
    count = 0;
    block_projection = false;
    rotation_optmization = true;
    last_rotation = 0.0f;
    block_count = 0;
    this->use_mean = false;
    max_proj_dist = 5.0;
    too_far = true;

    // Initialize Subscribers
    cloud_sub = nh->subscribe(pointcloud_topic, 1, &Projector::cloud_callback, this);
    boxes_sub = nh->subscribe(boxes_topic, 1, &Projector::boxes_callback, this);
    detection_flag_topic_sub = nh->subscribe(detection_flag_topic, 1, &Projector::flag_callback, this);
    if(rotation_optmization)
        odom_sub = nh->subscribe(odom_topic, 1, &Projector::odom_callback, this);

    // Initialize Publishers
    obj_pub = nh->advertise<custom_msgs::WorldObject>(out_topic, 1);
    obj_list_pub = nh->advertise<custom_msgs::ObjectList>(out_topic+"/list", 1);

    // Initialize Debug variables
    vis_pub = nh->advertise<visualization_msgs::Marker>( "raw_marker", 0 );
}

void Projector::flag_callback(const std_msgs::Int8 & flag)
{   
    try{
        listener->waitForTransform(global_frame, camera_frame, ros::Time::now(), ros::Duration(10.0) );
        listener->lookupTransform(global_frame, camera_frame, ros::Time(0), transform_buffer);
        listener->lookupTransform(global_frame, robot_frame, ros::Time(0), robot_transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("Error transforming point\n!");            
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Projector::odom_callback(const nav_msgs::Odometry & odom)
{
    double roll, pitch, yaw;
    auto ori = odom.pose.pose.orientation;
    tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    float rotation = yaw;
    if (std::abs(rotation - last_rotation) > 0.06)
    {
        block_projection = true; 
        block_count = 4;
    } 
    else
    {
        if (block_count > 1) 
        {
            block_projection = true;
            block_count--;
        }
        
        else block_projection = false;
    }

    //if(block_projection) ROS_INFO_STREAM("\nX");
    //else ROS_INFO_STREAM("\nAllow");

    last_rotation = rotation;
}

custom_msgs::WorldObject Projector::process_cloud(std::string class_name, pcl::PointCloud<pcl::PointXYZ> obj_cloud, int xmin, int xmax, int ymin, int ymax)
{
    custom_msgs::WorldObject obj;
    obj.objClass = class_name;

    pcl::PointCloud<pcl::PointXYZ> inlier_cloud;
    //pcl::fromROSMsg( cloud2, cloud);
    //cloud_buffer = pcl::PointCloud<pcl::PointXYZ>(cloud);

    // door
    if (class_name == "door")
    {
        // Apply RANSAC in camera_frame
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Optional
        seg.setOptimizeCoefficients (true);

        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.03);
        seg.setInputCloud (obj_cloud.makeShared());
        seg.segment (inliers, coefficients); 

/**     
 *      Publish inliers?
 * 
        inlier_cloud.width = inliers.indices.size();
        inlier_cloud.height = 1;
        inlier_cloud.resize(inlier_cloud.width);
        for(int i = 0; i < inliers.indices.size(); i++)
        {
            int index = inliers.indices[i];
            inlier_cloud.points[i] = obj_cloud.points[index];
        }
        sensor_msgs::PointCloud2 inliers_msg;
        pcl::toROSMsg(inlier_cloud, inliers_msg);
        inliers_msg.header.frame_id = global_frame;
        cloud_pub.publish(inliers_msg);
**/

        ROS_INFO_STREAM("\nInliers count: "+ std::to_string( inliers.indices.size()));
        pcl::PointXYZ obj_position;

        // 1. Calculates object location in camera_frame:         
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
        
        // Convert location and normal of object from camera_frame to global_frame
        /*
        tf::StampedTransform transform;
        try{
            ros::Time now = ros::Time::now();
            listener->waitForTransform(global_frame, camera_frame, now, ros::Duration(3.0));
            listener->lookupTransform(global_frame, camera_frame, ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("Error transforming point\n!");            
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        */
        // Rotation matrix and translation vector
        tf::Matrix3x3 rot_tf = transform.getBasis();       
        tf::Vector3 trans_tf = transform.getOrigin();
        
        // optional: use eigen for matrix multiplication?
        Eigen::Vector3d obj_pos_eigen(obj_position.x, obj_position.y, obj_position.z);
        Eigen::Matrix3d Rot;
        Eigen::Vector3d Trans;
        tf::matrixTFToEigen(rot_tf, Rot);
        tf::vectorTFToEigen(trans_tf, Trans);

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

        // Verify normal is pointing to correct direction (always away from robot)
        Eigen::Vector3d robot_normal(1.0, 0.0, 0.0);
        tf::Matrix3x3 rot_tf_robot = robot_transform.getBasis();
        Eigen::Matrix3d Rot_robot;
        tf::matrixTFToEigen(rot_tf_robot, Rot_robot);
        robot_normal = Rot_robot * robot_normal;
        if (robot_normal.dot(obj_normal) < 0) 
        {
            obj_normal *= -1;
        }

        // Plot normal
        if(coefficients.values.size() == 4)
        {
            pcl::PointXYZ start, end;
            start = obj_position;
            end = obj_position;
            end.x += obj_normal(0) * 0.8;
            end.y += obj_normal(1) * 0.8;
            end.z += obj_normal(2) * 0.8;

            //Mark in green or red 
            if (too_far)
              markArrow(start, end, global_frame, 1, 1.0, 0.0, 0.0);

            else 
              markArrow(start, end, global_frame, 1, 0.0, 1.0, 0.0);
        }

        // Calculates object angle in camera_frame (in XY plane):
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
        obj.prob = -1.0;
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
    count++; 
    transform = transform_buffer;
    ROS_INFO_STREAM("\nReceived box "+std::to_string( count));
    int box_num = boxes_ptr->bounding_boxes.size();

    custom_msgs::ObjectList objects_msg;
    objects_msg.num = 0;

    for(int i = 0; i < box_num; i++)
    {
        // Object class
        std::string class_name = boxes_ptr->bounding_boxes.at(i).Class;
        custom_msgs::WorldObject object; 

        // Object boundaries
        int max_width = cloud_buffer.width;
        int xmin = boxes_ptr->bounding_boxes.at(i).xmin+40;
        int ymin = boxes_ptr->bounding_boxes.at(i).ymin;
        int xmax = boxes_ptr->bounding_boxes.at(i).xmax+40;
        int ymax = boxes_ptr->bounding_boxes.at(i).ymax;

        if(xmin > max_width) xmin = max_width-1;
        if(xmax > max_width) xmax = max_width-1;

        // Crop pointcloud inside bounding box
        pcl::PointCloud<pcl::PointXYZ> cropped; 
        cropped.width = std::abs(xmax-xmin);
        cropped.height = std::abs(ymax-ymin);
        cropped.points.resize (cropped.width * cropped.height);

        for(int x = 0; x < cropped.width; x++)
            for(int y = 0; y < cropped.height; y++)
                cropped.at(x, y) = cloud_buffer.at(x+xmin, y+ymin);

        
        if(!block_projection)
        {
            // Process data in core function to generate a world object
            object = process_cloud(class_name, cropped, xmin, xmax, ymin, ymax);
            
            if(object.prob < 0) continue; 

            // Verify object is not too far away 
            float d = distanceFromRobot(object.x, object.y);
            if(d <= max_proj_dist)
            {
              // Publish encountered object
              too_far = false;              
              obj_pub.publish(object);
              objects_msg.objects.push_back(object);
              objects_msg.num += 1;
              ROS_INFO_STREAM("\nProcessed box "+std::to_string( count));
            }
            else
            {
              too_far = true;
              ROS_INFO_STREAM("\nToo faar away! Distance: "+std::to_string(d) + " > "+ std::to_string(max_proj_dist));
            }
        }
        else
        {
            ROS_INFO_STREAM("\nBlocked Projection!!");
        }
    }

    obj_list_pub.publish(objects_msg);
}

float Projector::distanceFromRobot(float x, float y)
{
  tf::Vector3 trans_robot = robot_transform.getOrigin();
  float dx = (trans_robot.getX() - x);
  float dy = (trans_robot.getY() - y);
  float dist = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
  return dist; 
}

void Projector::cloud_callback(const sensor_msgs::PointCloud2ConstPtr & cloud2_ptr)
{
    // Transform pointcloud frame 
    sensor_msgs::PointCloud2 cloud2;
    try{
        ros::Time now = ros::Time::now();
        listener->waitForTransform("camera_rgb_optical_frame", camera_frame, now, ros::Duration(10.0) );
        pcl_ros::transformPointCloud(camera_frame, *cloud2_ptr, cloud2, *listener);

    } catch (tf::TransformException ex) {
        ROS_ERROR("Error transforming cloud\n!");        
        ROS_ERROR("%s",ex.what());
    }

    // Transform pointcloud type from ros msg to pointcloud
    pcl::fromROSMsg( cloud2, cloud_buffer);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    /*
     * Publish pointcloud

    sensor_msgs::PointCloud2 cropped_msg;
    pcl::toROSMsg(cloud_buffer, cropped_msg);
    cropped_msg.header.frame_id = camera_frame;
    cloud_pub.publish(cropped_msg);
    */
}

void Projector::markArrow(pcl::PointXYZ start, pcl::PointXYZ end, std::string frame, int id, double r, double g, double b)
{
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> points(2);
    points[0].x = start.x;
    points[0].y = start.y;
    points[0].z = start.z+1.0;
    points[1].x = end.x;
    points[1].y = end.y;
    points[1].z = end.z+1.0;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "arrows";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.15;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; 
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.points = points;

    vis_pub.publish( marker );
}
