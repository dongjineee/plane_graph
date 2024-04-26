#include "qp_slam/keyframe.h"
#include <ros/ros.h>
#include <tf/tf.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/io/pcd_io.h>

#include <boost/serialization/serialization.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/format.hpp>

#include "math.h"

keyframe::keyframe(ros::NodeHandle nh) {
    get_params(nh);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &keyframe::odom_callback, this);
    pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pc_topic_, 1, &keyframe::pc_callback, this);
    image_sub_ = nh.subscribe<sensor_msgs::Image>(image_topic_, 1, &keyframe::img_callback, this);
    depth_sub_ = nh.subscribe<sensor_msgs::Image>(depth_topic_, 1, &keyframe::depth_callback, this);
    // 다른 발행자, 서비스 서버 초기화도 여기에서 수행합니다.

    odom_.setIdentity();

    // 필요한 초기화 코드 추가
}

void keyframe::get_params(ros::NodeHandle nh)
{
    // nh.param<string>("odom_topic", odom_topic_, "/odom");
    // nh.param<string>("pointcloud_topic", pc_topic_, "/velodyne_points/points");

    // nh.param<string>("odom_topic", odom_topic_, "/gt_odometry");

    nh.param<string>("odom_topic", odom_topic_, "/base_controller/odom");
    nh.param<string>("pointcloud_topic", pc_topic_, "/velodyne_points/points");
    nh.param<string>("image_topic", image_topic_, "/camera/color/image_raw");
    nh.param<string>("depth_topic", depth_topic_, "/project_image");

    // nh.param<string>("odom_topic", odom_topic_, "/platform/odometry");
    // nh.param<string>("pointcloud_topic", pc_topic_, "/platform/velodyne_points");

    pose_pub_ = nh.advertise<qp_slam::pose>("/key_",1);

    image_pub_ = nh.advertise<sensor_msgs::Image>("/test_3",1);
    depth_pub_ = nh.advertise<sensor_msgs::Image>("/depth_to",1);

    point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/depth_po",1);

}

void keyframe::depth_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    if(has_d_)
    {   has_d_ = false;
        depth_pub_.publish(*msg);
        point_pub_.publish(depth_point);
    }
}


void keyframe::img_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    if(has_pose_)
    {
        has_pose_ = false;

        _image = *msg;

        image_pub_.publish(_image);

        has_depth_ = true;
        // ROS_INFO("come image\n");
    }

}

void keyframe::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double radius = 0.0; // radius 초기화

    if(initial_set)
    {
        _odom.header.frame_id = msg->header.frame_id;
        _odom.header.stamp = msg->header.stamp;

        _odom.pose = msg->pose;
     
        // keyframe_odom_pub_.publish(_odom);

        // printf("odom_noise : {%f,%f,%f,%f,%f,%f,%f},\n", _odom.pose.pose.position.x,
        //                                 _odom.pose.pose.position.y,
        //                                 _odom.pose.pose.position.z,
        //                                 _odom.pose.pose.orientation.x,
        //                                 _odom.pose.pose.orientation.y,
        //                                 _odom.pose.pose.orientation.z,
        //                                 _odom.pose.pose.orientation.w
        //                                 );

        initial_set = false;

        has_key_ = true;
        has_d_ = true;
        has_pose_ = true;

    }

    if(!has_odom_)
    {
        odom_.translation() = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        has_odom_ = true;
    }

    else
    {
        double x_distance = pow((odom_.translation().x() - msg->pose.pose.position.x), 2);
        double y_distance = pow((odom_.translation().y() - msg->pose.pose.position.y), 2);
        radius = sqrt(x_distance + y_distance);
    }

    if(radius >= 0.0)
    {
        // nav_msgs::Odometry _odom;
        _odom.header.frame_id = msg->header.frame_id;
        _odom.header.stamp = msg->header.stamp;

        _odom.pose = msg->pose;
        
        // keyframe_odom_pub_.publish(_odom);
        
        // ROS_INFO("come odom");
        has_key_ = true;
        has_pose_ = true;
        has_d_ = true;
    }
}

void keyframe::pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    depth_point = *msg;
    if(has_key_)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);

        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = msg->header.stamp;

        qp_slam::pose pose_;

        pose_.pose_odom = _odom;
        pose_.pose_point_clouds = output;
        
        pose_pub_.publish(pose_);
        
        // keyframe_pc_pub_.publish(output);
        // keyframe_odom_pub_.publish(_odom);
        
        has_odom_ = false;
        has_key_ = false;


        ROS_INFO("come pc\n");
    }
}

// get_params, reset, odom_callback, pc_callback 등의 함수 구현도 활성화하고 필요에 따라 수정합니다.

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyframe_node");
    ros::NodeHandle nh;

    keyframe kf(nh); // 수정: NodeHandle 객체를 전달합니다.

    ros::spin();

    return 0;
}
