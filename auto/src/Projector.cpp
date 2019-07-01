#include "Projector.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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
    showClassName = true;
    quiet_mode = true;
    x_offset = 40; 

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
    cloud_pub = nh->advertise<sensor_msgs::PointCloud2>("segmentation_vis", 10);
    
}

// Check for frame transform every time a detection image is received
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

// Blocks projections if robot has rotated too fast, to avoid errors due to slow processing in real time scenarios. 
void Projector::odom_callback(const nav_msgs::Odometry & odom)
{
    // rotation_thresh: cannot rotate more than this amount, otherwise output is blocked for X frames.  
    double rotation_thresh = 0.09; 

    // blocked_frames: number of frames projection will be blocked when rotation is too large
    int blocked_frames = 4; 

    double roll, pitch, yaw;
    auto ori = odom.pose.pose.orientation;
    tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    float rotation = yaw;
    if (std::abs(rotation - last_rotation) > rotation_thresh)
    {
        block_projection = true; 
        block_count = blocked_frames;
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

    last_rotation = rotation;
}

custom_msgs::WorldObject Projector::process_cloud(std::string class_name, pcl::PointCloud<point_type> obj_cloud, int xmin, int xmax, int ymin, int ymax)
{
    // publish_inliers : used to visualize the inlier cloud detected. 
    bool publish_inliers = true;

    custom_msgs::WorldObject obj;
    obj.objClass = class_name;
    obj.prob = -1;
    
    pcl::PointCloud<point_type> inlier_cloud;
    

    // door
    if (class_name == "door")
    {
        

        obj.prob = 1;
        // Apply RANSAC in camera_frame
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;

        // Create the segmentation object
        pcl::SACSegmentation<point_type> seg;

        // Optional
        seg.setOptimizeCoefficients (true);

        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);
        seg.setInputCloud (obj_cloud.makeShared());
        seg.segment (inliers, coefficients); 
        if(publish_inliers)
        {
            inlier_cloud.width = inliers.indices.size();
            inlier_cloud.height = 1;
            inlier_cloud.resize(inlier_cloud.width);
            for(int i = 0; i < inliers.indices.size(); i++)
            {
                int index = inliers.indices[i];
                inlier_cloud.points[i] = obj_cloud.points[index];
            }
        }

        //if(!quiet_mode) ROS_INFO_STREAM("\nInliers count: "+ std::to_string( inliers.indices.size()));
        pcl::PointXYZ obj_position;

        // 1. Calculates object location in camera_frame:         
        // Object location is the mean of points inside bounding box
        if(method_door == 0)
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
        else if(method_door == 1)
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

        else {
            std::cout << "Unexpected method index for door!" << std::endl;
        }
        
        // 2. Convert oject location from camera frame to map frame
        pcl::PointXYZ position_final = Projector::convertToMapFrame(obj_position);

        // Get normal   
        Eigen::Matrix3d Rot;
        tf::matrixTFToEigen(transform.getBasis(), Rot);
        Eigen::Vector3d obj_normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        obj_normal = Rot * obj_normal;

        // Verify normal is pointing to correct direction (always away from robot)
        auto rpos = robot_transform.getOrigin();
        Eigen::Vector3d robot_position(rpos.getX(), rpos.getY(), rpos.getZ()); 
        Eigen::Vector3d obj_start(position_final.x, position_final.y, position_final.z );
        Eigen::Vector3d obj_end(position_final.x+obj_normal(0), position_final.y+obj_normal(1), position_final.z+obj_normal(2));

        Eigen::Vector3d d1 = obj_start - robot_position; 
        Eigen::Vector3d d2 = obj_end - robot_position; 

        double dstart = d1.dot(d1);
        double dend = d2.dot(d2);

        if(dstart < dend)
        {
            obj_normal *= -1;
        }
       
        // Calculates object angle in camera_frame (in XY plane):
        // It is the angle of -normal in XY plane (Z is upwards in usual map frame)
        float x = obj_normal(0);
        float y = obj_normal(1);
        double angle = atan2(y, x);
        obj.x = position_final.x;
        obj.y = position_final.y;
        obj.angle = angle;
    }

    else if (class_name == "bench" || class_name == "fire")
    {
        obj.prob = 1;
        pcl::PointXYZ position; 
        int minimum_cluster_size = 4;
        int method = 2; 

        // Naive approach: get the mean value at the center with a square window
        if(method_bench == 0)
        {
            
            double meanx = 0;
            double meany = 0;
            double meanz = 0;
            unsigned x_min =  obj_cloud.width/2 - (window_width/2);
            unsigned y_min = obj_cloud.height/2 - (window_width/2);

            for(int i = x_min; i < x_min + window_width; i++)
                for(int j = y_min; j< y_min + window_width; j++)
                {
                    point_type point = obj_cloud.at(i, j);
                    meanx += point.x / (window_width*window_width);
                    meany += point.y / (window_width*window_width);
                    meanz += point.z / (window_width*window_width);
                }

            if(publish_inliers)
            {
                inlier_cloud = obj_cloud; 
            }

            position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
        }

        // Remove planes and then apply clustering 
        else if(method_bench == 1)
        {
            pcl::PointCloud<point_type>::Ptr cloud (obj_cloud.makeShared());

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<point_type> vg;
            pcl::PointCloud<point_type>::Ptr cloud_filtered (new pcl::PointCloud<point_type>), cloud_f (new pcl::PointCloud<point_type>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);

            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<point_type> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<point_type>::Ptr cloud_plane (new pcl::PointCloud<point_type> ());
            pcl::PCDWriter writer;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);
            
            int nr_points = (int) cloud_filtered->points.size ();
            
            // Remove NaNs
            cloud_filtered->is_dense = false;
            std::vector< int > a; 
            pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, a);
            
            if(cloud_filtered->size() < minimum_cluster_size)
            {
                // Not enough points
                obj.prob = -1; 
            }
            else
            {
                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type>);
                tree->setInputCloud (cloud_filtered);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<point_type> ec;
                ec.setClusterTolerance (0.5); // 50cm
                ec.setMinClusterSize (minimum_cluster_size);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_filtered);
                ec.extract (cluster_indices);
                if(!quiet_mode) ROS_INFO_STREAM("\nclass: "+ obj.objClass + ", clusters: " +std::to_string( cluster_indices.size()));

                int biggest_cluster_size = 0; 
                int biggest_cluster_index = 0;

                if(cluster_indices.size() == 0)
                {
                    obj.prob = -1;
                }
                else
                {
                    for (int i = 0; i < cluster_indices.size(); i++)
                    {
                        auto indices = cluster_indices.at(i).indices;
                        if(indices.size() > biggest_cluster_size )
                        {
                            biggest_cluster_index = i;
                        }
                    }

                    auto indices = cluster_indices.at(biggest_cluster_index).indices;

                    // Find center of cluster
                    double meanx = 0;
                    double meany = 0;
                    double meanz = 0;

                    for (int i = 0; i < indices.size(); i++ )
                    {
                        int index = indices.at(i);
                        point_type point = cloud_filtered->points[index]; 
                        
                        meanx += point.x / (indices.size());
                        meany += point.y / (indices.size());
                        meanz += point.z / (indices.size());

                        if(publish_inliers) inlier_cloud.push_back(point);
                    }
                    inlier_cloud.width = indices.size();
                    inlier_cloud.height = 1; 
                    inlier_cloud.is_dense = true;

                    position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
                }
            }

        }

        // Simple cluster 
        else if (method_bench == 2)
        {
            pcl::PointCloud<point_type>::Ptr cloud (obj_cloud.makeShared());

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<point_type> vg;
            pcl::PointCloud<point_type>::Ptr cloud_filtered (new pcl::PointCloud<point_type>), cloud_f (new pcl::PointCloud<point_type>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);

            // (Do not remove planes from model ...)
            
            // Remove NaNs
            cloud_filtered->is_dense = false;
            std::vector< int > a; 
            pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, a);
            
            if(cloud_filtered->size() < minimum_cluster_size)
            {
                // Not enough points
                obj.prob = -1; 
            }
            else
            {
                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type>);
                tree->setInputCloud (cloud_filtered);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<point_type> ec;
                ec.setClusterTolerance (0.2); // 20cm
                ec.setMinClusterSize (minimum_cluster_size);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_filtered);
                ec.extract (cluster_indices);
                if(!quiet_mode) ROS_INFO_STREAM("\nclass: "+ obj.objClass + ", clusters: " +std::to_string( cluster_indices.size()));

                int biggest_cluster_size = 0; 
                int biggest_cluster_index = 0;

                if(cluster_indices.size() == 0)
                {
                    obj.prob = -1;
                }
                else
                {
                    // Find cluster with most inliers
                    for (int i = 0; i < cluster_indices.size(); i++)
                    {
                        auto indices = cluster_indices.at(i).indices;
                        if(indices.size() > biggest_cluster_size )
                        {
                            biggest_cluster_index = i;
                        }
                    }

                    auto indices = cluster_indices.at(biggest_cluster_index).indices;

                    // Find center of cluster
                    double meanx = 0;
                    double meany = 0;
                    double meanz = 0;

                    for (int i = 0; i < indices.size(); i++ )
                    {
                        int index = indices.at(i);
                        point_type point = cloud_filtered->points[index]; 
                        
                        meanx += point.x / (indices.size());
                        meany += point.y / (indices.size());
                        meanz += point.z / (indices.size());

                        if(publish_inliers) inlier_cloud.push_back(point);
                    }
                    inlier_cloud.width = indices.size();
                    inlier_cloud.height = 1; 
                    inlier_cloud.is_dense = true;

                    position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
                }

            }
        }

        else{
            std::cout << "Unexpected method index for door!" << std::endl;
        }
        
        // Set position back from eigen
        obj.x = position.x;
        obj.y = position.y;
        obj.angle = 0;
    }

    else if (class_name == "water" || class_name == "trash")
    {
        obj.prob = 1;
        pcl::PointXYZ position; 
        int minimum_cluster_size = 4;
        int method = 2; 

        // Naive approach: get the mean value at the center with a square window
        if(method_water == 0)
        {
            double meanx = 0;
            double meany = 0;
            double meanz = 0;
            unsigned x_min =  obj_cloud.width/2 - (window_width/2);
            unsigned y_min = obj_cloud.height/2 - (window_width/2);

            for(int i = x_min; i < x_min + window_width; i++)
                for(int j = y_min; j< y_min + window_width; j++)
                {
                    point_type point = obj_cloud.at(i, j);
                    meanx += point.x / (window_width*window_width);
                    meany += point.y / (window_width*window_width);
                    meanz += point.z / (window_width*window_width);
                }

            if(publish_inliers)
            {
                inlier_cloud = obj_cloud; 
            }

            position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
        }

        // Remove planes and then apply clustering 
        else if(method_water == 1)
        {
            pcl::PointCloud<point_type>::Ptr cloud (obj_cloud.makeShared());

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<point_type> vg;
            pcl::PointCloud<point_type>::Ptr cloud_filtered (new pcl::PointCloud<point_type>), cloud_f (new pcl::PointCloud<point_type>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);

            // Create the segmentation object for the planar model and set all the parameters
            pcl::SACSegmentation<point_type> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<point_type>::Ptr cloud_plane (new pcl::PointCloud<point_type> ());
            pcl::PCDWriter writer;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (100);
            seg.setDistanceThreshold (0.02);
            
            int nr_points = (int) cloud_filtered->points.size ();

            /*
            while (cloud_filtered->points.size () > 0.98 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud (cloud_filtered);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                {
                    std::cout << "Could not estimate a planar model for the given pointcloud." << std::endl;
                    break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<point_type> extract;
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers);
                extract.setNegative (false);

                // Get the points associated with the planar surface
                extract.filter (*cloud_plane);
                
                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_f);
                *cloud_filtered = *cloud_f;
            }
            */
            // Remove NaNs
            cloud_filtered->is_dense = false;
            std::vector< int > a; 
            pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, a);
            
            if(cloud_filtered->size() < minimum_cluster_size)
            {
                // Not enough points
                obj.prob = -1; 
            }
            else
            {
                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type>);
                tree->setInputCloud (cloud_filtered);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<point_type> ec;
                ec.setClusterTolerance (0.5); // 50cm
                ec.setMinClusterSize (minimum_cluster_size);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_filtered);
                ec.extract (cluster_indices);
                if(!quiet_mode) ROS_INFO_STREAM("\nclass: "+ obj.objClass + ", clusters: " +std::to_string( cluster_indices.size()));

                int biggest_cluster_size = 0; 
                int biggest_cluster_index = 0;

                if(cluster_indices.size() == 0)
                {
                    obj.prob = -1;
                }
                else
                {
                    for (int i = 0; i < cluster_indices.size(); i++)
                    {
                        auto indices = cluster_indices.at(i).indices;
                        if(indices.size() > biggest_cluster_size )
                        {
                            biggest_cluster_index = i;
                        }
                    }

                    auto indices = cluster_indices.at(biggest_cluster_index).indices;

                    // Find center of cluster
                    double meanx = 0;
                    double meany = 0;
                    double meanz = 0;

                    for (int i = 0; i < indices.size(); i++ )
                    {
                        int index = indices.at(i);
                        point_type point = cloud_filtered->points[index]; 
                        
                        meanx += point.x / (indices.size());
                        meany += point.y / (indices.size());
                        meanz += point.z / (indices.size());

                        if(publish_inliers) inlier_cloud.push_back(point);
                    }
                    inlier_cloud.width = indices.size();
                    inlier_cloud.height = 1; 
                    inlier_cloud.is_dense = true;

                    position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
                }
            }

        }

        // Simple cluster 
        else if (method_water == 2)
        {
            pcl::PointCloud<point_type>::Ptr cloud (obj_cloud.makeShared());

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<point_type> vg;
            pcl::PointCloud<point_type>::Ptr cloud_filtered (new pcl::PointCloud<point_type>), cloud_f (new pcl::PointCloud<point_type>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.01f, 0.01f, 0.01f);
            vg.filter (*cloud_filtered);

            // (Do not remove planes from model ...)
            
            // Remove NaNs
            cloud_filtered->is_dense = false;
            std::vector< int > a; 
            pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, a);
            
            if(cloud_filtered->size() < minimum_cluster_size)
            {
                // Not enough points
                obj.prob = -1; 
            }
            else
            {
                // Creating the KdTree object for the search method of the extraction
                pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type>);
                tree->setInputCloud (cloud_filtered);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<point_type> ec;
                ec.setClusterTolerance (0.2); // 20cm
                ec.setMinClusterSize (minimum_cluster_size);
                ec.setMaxClusterSize (25000);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud_filtered);
                ec.extract (cluster_indices);
                if(!quiet_mode) ROS_INFO_STREAM("\nclass: "+ obj.objClass + ", clusters: " +std::to_string( cluster_indices.size()));

                int biggest_cluster_size = 0; 
                int biggest_cluster_index = 0;

                if(cluster_indices.size() == 0)
                {
                    obj.prob = -1;
                }
                else
                {
                    // Find cluster with most inliers
                    for (int i = 0; i < cluster_indices.size(); i++)
                    {
                        auto indices = cluster_indices.at(i).indices;
                        if(indices.size() > biggest_cluster_size )
                        {
                            biggest_cluster_index = i;
                        }
                    }

                    auto indices = cluster_indices.at(biggest_cluster_index).indices;

                    // Find center of cluster
                    double meanx = 0;
                    double meany = 0;
                    double meanz = 0;

                    for (int i = 0; i < indices.size(); i++ )
                    {
                        int index = indices.at(i);
                        point_type point = cloud_filtered->points[index]; 
                        
                        meanx += point.x / (indices.size());
                        meany += point.y / (indices.size());
                        meanz += point.z / (indices.size());

                        if(publish_inliers) inlier_cloud.push_back(point);
                    }
                    inlier_cloud.width = indices.size();
                    inlier_cloud.height = 1; 
                    inlier_cloud.is_dense = true;

                    position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
                }

            }
        }

        else{
            std::cout << "Unexpected method index for water!" << std::endl;
        }
        
        // Set position back from eigen
        obj.x = position.x;
        obj.y = position.y;
        obj.angle = 0;
    }

    else if (class_name == "person")
    {
        obj.prob = 1;
        pcl::PointXYZ position; 
        int minimum_cluster_size = 10;

        pcl::PointCloud<point_type>::Ptr cloud (obj_cloud.makeShared());

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<point_type> vg;
        pcl::PointCloud<point_type>::Ptr cloud_filtered (new pcl::PointCloud<point_type>), cloud_f (new pcl::PointCloud<point_type>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);

        // (Do not remove planes from model ...)
        
        // Remove NaNs
        cloud_filtered->is_dense = false;
        std::vector< int > a; 
        pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, a);
        
        if(cloud_filtered->size() < minimum_cluster_size)
        {
            // Not enough points
            obj.prob = -1; 
        }
        else
        {
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<point_type> ec;
            ec.setClusterTolerance (0.2); // 20cm
            ec.setMinClusterSize (minimum_cluster_size);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);
            if(!quiet_mode) ROS_INFO_STREAM("\nclass: "+ obj.objClass + ", clusters: " +std::to_string( cluster_indices.size()));

            int biggest_cluster_size = 0; 
            int biggest_cluster_index = 0;

            if(cluster_indices.size() == 0)
            {
                obj.prob = -1;
            }
            else
            {
                // Find cluster with most inliers
                for (int i = 0; i < cluster_indices.size(); i++)
                {
                    auto indices = cluster_indices.at(i).indices;
                    if(indices.size() > biggest_cluster_size )
                    {
                        biggest_cluster_index = i;
                    }
                }

                auto indices = cluster_indices.at(biggest_cluster_index).indices;

                // Find center of cluster
                double meanx = 0;
                double meany = 0;
                double meanz = 0;

                for (int i = 0; i < indices.size(); i++ )
                {
                    int index = indices.at(i);
                    point_type point = cloud_filtered->points[index]; 
                    
                    meanx += point.x / (indices.size());
                    meany += point.y / (indices.size());
                    meanz += point.z / (indices.size());

                    if(publish_inliers) inlier_cloud.push_back(point);
                }
                inlier_cloud.width = indices.size();
                inlier_cloud.height = 1; 
                inlier_cloud.is_dense = true;

                position = Projector::convertToMapFrame(pcl::PointXYZ(meanx, meany, meanz));
            }

        }
        
        // Set position back from eigen
        obj.x = position.x;
        obj.y = position.y;
        obj.angle = 0;
    }

    if(publish_inliers)
    {
        // Convert pointcloud to ROS msg
        sensor_msgs::PointCloud2 inliers_msg;
        pcl::toROSMsg(inlier_cloud, inliers_msg);
        inliers_msg.header.frame_id = camera_frame;

        // Convert from camera_frame to global_frame
        // pcl_ros::transformPointCloud(global_frame, transform, inliers_msg, inliers_msg);

        // Publish
        cloud_pub.publish(inliers_msg);
    }

    else
    {
        if(!quiet_mode) ROS_INFO_STREAM("\nUnimplemented");
        obj.prob = -1.0;
    }

    return obj;
}

void Projector::boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & boxes_ptr)
{
    count++; 
    transform = transform_buffer;
    if(!quiet_mode) ROS_INFO_STREAM("\nProcessing "+std::to_string(count)+ " objects. ");
    int box_num = boxes_ptr->bounding_boxes.size();

    custom_msgs::ObjectList objects_msg;
    objects_msg.num = 0;

    // Process each bounding box
    for(int i = 0; i < box_num; i++)
    {
        if(boxes_ptr->bounding_boxes.at(i).Class != "door")
            if(!quiet_mode) ROS_INFO_STREAM("\n\nProcessing " + boxes_ptr->bounding_boxes.at(i).Class + ": ");

        // Object class
        std::string class_name = boxes_ptr->bounding_boxes.at(i).Class;
        custom_msgs::WorldObject object; 

        // Object boundaries
        int max_width = cloud_buffer.width;
        int xmin = boxes_ptr->bounding_boxes.at(i).xmin+x_offset;
        int ymin = boxes_ptr->bounding_boxes.at(i).ymin;
        int xmax = boxes_ptr->bounding_boxes.at(i).xmax+x_offset;
        int ymax = boxes_ptr->bounding_boxes.at(i).ymax;

        if(xmin > max_width) xmin = max_width-1;
        if(xmax > max_width) xmax = max_width-1;

        // Crop pointcloud inside bounding box
        pcl::PointCloud<point_type> cropped; 
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
            
            // Could not process object
            if(object.prob < 0) {
                if(!quiet_mode) ROS_INFO_STREAM("Could not process "+ class_name);
                continue; 
            }

            // Verify object is not too far away 
            float d = distanceFromRobot(object.x, object.y);
            if(d <= max_proj_dist)
            {
              // Publish encountered object
              too_far = false;              
              obj_pub.publish(object);
              objects_msg.objects.push_back(object);
              objects_msg.num += 1;

              if(boxes_ptr->bounding_boxes.at(i).Class != "door")
                if(!quiet_mode) ROS_INFO_STREAM("\n"+ object.objClass + " successful!");
            }
            else
            {
              too_far = true;
              if(!quiet_mode) ROS_INFO_STREAM("\n" + object.objClass + " too faar away!\nDistance: "+std::to_string(d) + " > "+ std::to_string(max_proj_dist));
            }
        }
        else
        {
            if(boxes_ptr->bounding_boxes.at(i).Class != "door")
                if(!quiet_mode) ROS_INFO_STREAM("\nBlocked Projection! (Robot rotating too fast...)");
        }
    }

    // Publish list of objects found in that frame
    obj_list_pub.publish(objects_msg);

    // Plot objects found in that frame
    for(int i = 0; i < objects_msg.objects.size(); i++)
    {
        pcl::PointXYZ position;
        custom_msgs::WorldObject obj = objects_msg.objects[i];
        position.x = obj.x;
        position.y = obj.y;
        position.z = 1.0;

        if(obj.prob > 0) 
        {
            if(obj.objClass == "door")
            {
                markObject(position, obj.objClass, global_frame, "cube", i+100, 0.3, 0.3, 0.8, 0.0, 1.0, 0.0);
            }

            else if (obj.objClass == "bench")
            {
                markObject(position, obj.objClass, global_frame, "cube", i+100, 0.3, 0.3, 0.5, 0.0, 0.1, 0.8);
            }

            else if (obj.objClass == "fire")
            {
                markObject(position, obj.objClass, global_frame, "cube", i+100, 0.3, 0.3, 0.5, 1.0, 0.1, 0.1);
            }
            else if (obj.objClass == "water")
            {
                markObject(position, obj.objClass, global_frame, "cube", i+100, 0.3, 0.3, 0.5, 0.4, 0.8, 1.0);
            }
            else if (obj.objClass == "trash")
            {
                position.z = 0.2;
                markObject(position, obj.objClass, global_frame, "cube", i+100, 0.4, 0.4, 0.3, 0.8, 0.7, 0.1);
            }

            else if (obj.objClass == "person")
            {
                position.z = 1.1;
                markObject(position, obj.objClass, global_frame, "cylinder", i+100, 0.4, 0.4, 1.6, 0.9, 0.5, 0.1);
            }
        }
        
    }

    if(!quiet_mode) ROS_INFO_STREAM("\n------------------------\n"); 
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
        listener->waitForTransform(camera_frame, "camera_rgb_optical_frame", now, ros::Duration(10.0) );
        pcl_ros::transformPointCloud(camera_frame, *cloud2_ptr, cloud2, *listener);

    } catch (tf::TransformException ex) {
        ROS_ERROR("Error transforming cloud\n!");        
        ROS_ERROR("%s",ex.what());
    }

    // Transform pointcloud type from ros msg to pointcloud
    pcl::fromROSMsg( cloud2, cloud_buffer);
}

pcl::PointXYZ Projector::convertToMapFrame(const pcl::PointXYZ & point)
{
    pcl::PointXYZ result; 
    tf::Matrix3x3 rot_tf = transform.getBasis();       
    tf::Vector3 trans_tf = transform.getOrigin();
    
    // optional: use eigen for matrix multiplication
    Eigen::Vector3d obj_pos_eigen(point.x, point.y, point.z);
    Eigen::Matrix3d Rot;
    Eigen::Vector3d Trans;
    tf::matrixTFToEigen(rot_tf, Rot);
    tf::vectorTFToEigen(trans_tf, Trans);

    // Rotate and translate position
    obj_pos_eigen = Rot * obj_pos_eigen;
    obj_pos_eigen = obj_pos_eigen + Trans;
    
    // Set position back from eigen
    result.x = obj_pos_eigen(0);
    result.y = obj_pos_eigen(1);
    result.z = obj_pos_eigen(2);

    return result; 
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

void Projector::markObject(const pcl::PointXYZ & location, const std::string & className, const std::string & frame, const std::string & shape, int id, double sx, double sy, double sz, double r, double g, double b)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(2,0);
    marker.ns = className;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    if(shape == "cylinder") marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.a = 0.7; 
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    
    marker.pose.position.x = location.x;
    marker.pose.position.y = location.y;
    marker.pose.position.z = location.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    vis_pub.publish( marker );

    if(showClassName)
    {
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration(2,0);
        marker.ns = className+"_text";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0;
        marker.scale.y = 0;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; 
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        
        marker.pose.position.x = location.x;
        marker.pose.position.y = location.y;
        marker.pose.position.z = location.z+sz;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.text = className;

        vis_pub.publish( marker );
    }
}
