#include <ros/ros.h>


// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>
// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/OrientedPlane3Factor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include <string>

#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

#include <gtsam/inference/Symbol.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "qp_slam/plane.h"
#include "qp_slam/planes.h"
#include "qp_slam/key_m.h"
#include "qp_slam/pose.h"

#include "quadric_msgs/DualQuadricBox.h"

#include <pcl/common/transforms.h> 
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/segmentation/extract_clusters.h>
#include <random>

#include <pcl/features/normal_3d.h>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>
#include <gtsam/base/Vector.h>



using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::P;
using namespace std;

// using gtsam::symbol_shorthand::X;
// using gtsam::symbol_shorthand::P;

class backend
{
    public:
        backend(ros::NodeHandle nh); 
        ~backend(){}

        gtsam::Pose3 getPoseFromOdomList(const std::vector<std::vector<float>>& odom_list, int k);
        
        void visualizeResults(const gtsam::Values& results);
        void get_params(ros::NodeHandle nh);
        void plane_callback(const qp_slam::planes::ConstPtr& msg);

        void pose_callback(const qp_slam::pose::ConstPtr& msg);

        void terminal_view(const qp_slam::planes::ConstPtr& msg); 
        pcl::PointCloud<pcl::PointXYZ> filter_cloud(const sensor_msgs::PointCloud2ConstPtr& input, float filterRes);

    private:

        ros::Publisher marker_pub_; // 시각화 마커 발행을 위한 publisher
        std::vector<gtsam::OrientedPlane3> plane_set; // 평면 집합
        ros::Subscriber plane_sub;
        ros::Publisher points_pub_;
        ros::Publisher points_pub_2;
        ros::Publisher points_pub_3;

        ros::Publisher key_pub;

        string plane_topic_;
        string key_topic;

        int odom_id = -1;
        int plane_id;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;
        gtsam::Pose3 pre_pose;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr all_pointclouds(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ> all_pointclouds;
        // pcl::PointCloud<pcl::PointXYZ> all_pointclouds_1;
        pcl::PointCloud<pcl::PointXYZ> all_pointclouds_3;
        bool flag = true;
        int cnt = 0;
        float sensor_height_ = 0.08;
        std::vector<sensor_msgs::PointCloud2> map_cloud;

        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        ros::Publisher odom_pub;

        ros::Subscriber key_sub;
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
        gtsam::noiseModel::Diagonal::shared_ptr x_p_noise;
}; 

