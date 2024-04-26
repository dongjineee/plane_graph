#pragma once

#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "qp_slam/pose.h"


using namespace std;

class keyframe
{
public:
    keyframe(ros::NodeHandle nh);
    ~keyframe() {}

    void get_params(ros::NodeHandle nh);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void img_callback(const sensor_msgs::Image::ConstPtr& msg);
    void depth_callback(const sensor_msgs::Image::ConstPtr& msg);

    // bool map_save(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    
private:
    void reset();

    string odom_topic_;
    string pc_topic_;
    string frame_id_;
    string image_topic_;
    string depth_topic_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber pc_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    
    ros::Publisher keyframe_odom_pub_;
    ros::Publisher keyframe_pc_pub_;

    ros::Publisher pose_pub_;

    ros::Publisher sampled_pc_pub_;

    ros::Publisher last_pc_pub_;

    ros::Publisher image_pub_;
    ros::Publisher depth_pub_;
    ros::Publisher point_pub_;

    sensor_msgs::PointCloud2 depth_point;
    bool has_odom_ = false;
    bool has_key_ = false;

    bool has_image_ = false;
    bool has_pose_ = false;
    bool has_d_ = false;

    bool has_depth_ = false;

    bool initial_set = true;

    Eigen::Isometry3d key_odom_;
    float x_distance;
    float y_distance;

    float radius;

    Eigen::Isometry3d odom_;
    double latest_odom_time_;
    Eigen::Matrix4d laser_offset_mat;

    tf::TransformBroadcaster br_;

    bool is_pub_keyposes_;
    bool is_pub_last_pc_;

    sensor_msgs::PointCloud2 keyframe_pc;

    nav_msgs::Odometry _odom;
    sensor_msgs::Image _image;
    sensor_msgs::Image _depth;

    void pub_keyposes(const ros::Time& stamp, const string& frame_id);
};


