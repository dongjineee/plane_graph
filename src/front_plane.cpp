#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/segmentation/extract_clusters.h>
#include <random>
#include <tf/tf.h> 
#include <tf/transform_listener.h> 

#include <pcl/features/normal_3d.h>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <pcl/filters/voxel_grid.h>

#include "qp_slam/plane.h"
#include "qp_slam/planes.h"
#include "qp_slam/key_m.h"

#include <boost/make_shared.hpp> // boost 라이브러리를 include해야 합니다.


namespace fs = boost::filesystem;

class graph {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber keyframe_pose_sub_;
    ros::Subscriber gtodom_sub_;
    ros::Publisher pub_1;
    ros::Publisher pub_2;
    ros::Publisher pub_3;

    std::vector<ros::Publisher> test_cloud;

    ros::Publisher segmented_cloud_pub;
    
    ros::Publisher x_pub;
    ros::Publisher y_pub;

    ros::Publisher t_1;
    ros::Publisher t_2;
    ros::Publisher t_3;
    ros::Publisher t_4;
    ros::Publisher t_5;

    ros::Publisher all;
    ros::Publisher x_all;

    std::string frame_id;
    int min_seg_points_;
    bool use_euclidean_filter;
    bool use_shadow_filter; // 필요한 경우 추가

    #define X_VERT_PLANE 0
    #define Y_VERT_PLANE 1
    #define HORT_PLANE 2

    #define max_time 200

    int tt_cnt = 0;

    double plane_dist_threshold = 0.35;

    int cnt = 0;
    std::vector<uint16_t> parameter_vector;
    pcl::PointCloud<pcl::PointNormal>::Ptr accumulated_cloud ; 
    pcl::PointCloud<pcl::PointNormal>::Ptr accumulated_cloud_x ;
 
    ros::Publisher planes_pub;

    qp_slam::planes backend_planes;

    bool initialize = true; 

    int o_cnt = 0;
    int v_cnt = 0;
    nav_msgs::Odometry odom_;
    nav_msgs::Odometry odom_gt;
    nav_msgs::Odometry odom_noise;

        class Planes{
        public:
            int id; // asssociation group
            gtsam::Vector4 plane; /// global normal
            gtsam::Vector4 plane_body; // local normal

            pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_seg_body;  // segmented points of the plane in local body frame

            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
                cloud_seg_body_vec;  // vector of segmented points of the plane in local

                                    // body frame
            pcl::PointCloud<pcl::PointNormal>::Ptr
                cloud_seg_map;           // segmented points of the plane in global map frame

            Eigen::Matrix3d covariance;  // covariance of the landmark

            gtsam::Vector4 plane_node; 

            int revit_id;
            std::string type;                         // Type online or prior
            double length;                            // Length of plane
            bool matched = false;                     // Flag if matched with prior/online or not
            Eigen::Vector2d start_point =
                Eigen::Vector2d::Ones();  // start point of the PRIOR wall in revit
            Eigen::Vector3d wall_point;   // point used to calculate prior wall center
            bool on_wall = false;  // variable to check if a plane is already associated to a wall


    };


    class VerticalPlanes : public Planes {
        public:
            VerticalPlanes() : Planes() {}
            ~VerticalPlanes() {}
    };

    // HorizontalPlanes 구조체 정의
    struct HorizontalPlanes : public Planes {
        public:
            HorizontalPlanes() : Planes() {}
            ~HorizontalPlanes() {}
    };
    
        std::unordered_map<int, VerticalPlanes> x_planes;
        std::unordered_map<int, VerticalPlanes> y_planes;
        std::unordered_map<int, HorizontalPlanes> hort_planes;

public:
    // 클래스 생성자
    graph() {
        // pc_sub_ = nh_.subscribe("/velodyne_points/points", 1, &graph::pointCloud2Callback, this);
        // // odom_sub_ = nh_.subscribe("/gt_odometry", 1, &graph::odomCallback, this);
        // odom_sub_ = nh_.subscribe("/base_controller/odom", 1, &graph::odomCallback, this);

        pc_sub_ = nh_.subscribe("/velodyne_points111", 1, &graph::pointCloud2Callback, this);
        odom_sub_ = nh_.subscribe("/base_controller/odom", 1, &graph::odomCallback, this);
        gtodom_sub_ = nh_.subscribe("/gt_odometry", 1, &graph::odomCallback_gt, this);
        keyframe_pose_sub_ = nh_.subscribe("/key_pose", 1, &graph::pose_callback, this);  


        // gtodom_sub_ = nh_.subscribe("/base_controller/odom", 1, &graph::odomCallback_gt, this);

        pub_1 = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1);
        pub_2 = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud_1", 1);
        pub_3 = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_cloud_2", 1);

        test_cloud.resize(5);

        test_cloud[0] = nh_.advertise<sensor_msgs::PointCloud2>("/test1", 1);
        test_cloud[1] = nh_.advertise<sensor_msgs::PointCloud2>("/test2", 1);
        test_cloud[2] = nh_.advertise<sensor_msgs::PointCloud2>("/test3", 1);
        test_cloud[3] = nh_.advertise<sensor_msgs::PointCloud2>("/test4", 1);
        test_cloud[4] = nh_.advertise<sensor_msgs::PointCloud2>("/test5", 1);

        segmented_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);

        x_pub = nh_.advertise<sensor_msgs::PointCloud2>("/test10", 1);
        y_pub = nh_.advertise<sensor_msgs::PointCloud2>("/test11", 1);


        t_1 = nh_.advertise<sensor_msgs::PointCloud2>("/t_1", 1);
        t_2 = nh_.advertise<sensor_msgs::PointCloud2>("/t_2", 1);
        t_3 = nh_.advertise<sensor_msgs::PointCloud2>("/t_3", 1);
        t_4 = nh_.advertise<sensor_msgs::PointCloud2>("/t_4", 1);
        t_5 = nh_.advertise<sensor_msgs::PointCloud2>("/t_5", 1);

        all = nh_.advertise<sensor_msgs::PointCloud2>("/all", 1);
        x_all = nh_.advertise<sensor_msgs::PointCloud2>("/x_all", 1);

        planes_pub = nh_.advertise<qp_slam::planes>("plane", 1);

        nh_.param<std::string>("frame_id", frame_id, "test");

        use_euclidean_filter = false; // 필요에 따라 초기화
        use_shadow_filter = false; // 필요에 따라 초기화

        parameter_vector = Iteration_initial(max_time);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected_2(new pcl::PointCloud<pcl::PointNormal>());
        accumulated_cloud_x = cloud_seg_detected_2;

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected_1(new pcl::PointCloud<pcl::PointNormal>());
        accumulated_cloud = cloud_seg_detected_1;

    }

    nav_msgs::Odometry store_odom(std::vector<std::vector<float>> list, int cnt)
    {
        nav_msgs::Odometry odom;

        odom.pose.pose.position.x = list[cnt][0];
        odom.pose.pose.position.y = list[cnt][1];
        odom.pose.pose.position.z = list[cnt][2];

        odom.pose.pose.orientation.x = list[cnt][3];
        odom.pose.pose.orientation.y = list[cnt][4];
        odom.pose.pose.orientation.z = list[cnt][5];
        odom.pose.pose.orientation.w = list[cnt][6];
        
        return odom;
    }

    std::vector<uint16_t> Iteration_initial(uint16_t time)
    {
        std::vector<uint16_t> plane_time;

        for(int i = 0; i < 5; i++) plane_time.push_back(time);

        return plane_time;
    } // initial ransac_number parmeter


    void pose_callback(const qp_slam::key_m::ConstPtr& msg)
    {

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(msg->key_point_clouds, pcl_cloud);

        backend_planes = qp_slam::planes();
        odom_ = msg->key_odom;
        // backend_planes.odom = odom_;
        backend_planes.point_clouds = msg->key_point_clouds;
        // printf("{");
        printf("ff");
        front_end(pcl_cloud);
        // printf("},\n\n");
        printf("\n");
        cnt = 0;
        o_cnt++;

        topic_publish(*accumulated_cloud_x, x_all);
        topic_publish(*accumulated_cloud, all);
        accumulated_cloud_x = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        accumulated_cloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        if(initialize) initialize = false;
        planes_pub.publish(backend_planes);

    }

     // PointCloud2 메시지 콜백 함수
    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        tt_cnt = 0;

        v_cnt ++;

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*input_cloud, pcl_cloud);

        // ROS_INFO("---------------------------%d-------------------------------", v_cnt);
        // printf("\n");


        // printf("noise_odom : {%f,%f,%f,%f,%f,%f,%f},\n", odom_.pose.pose.position.x,
        //                                 odom_.pose.pose.position.y,
        //                                 odom_.pose.pose.position.z,
        //                                 odom_.pose.pose.orientation.x,
        //                                 odom_.pose.pose.orientation.y,
        //                                 odom_.pose.pose.orientation.z,
        //                                 odom_.pose.pose.orientation.w
        //                                 );

        // printf("gt_odom : {%f,%f,%f,%f,%f,%f,%f},\n", odom_gt.pose.pose.position.x,
        //                                 odom_gt.pose.pose.position.y,
        //                                 odom_gt.pose.pose.position.z,
        //                                 odom_gt.pose.pose.orientation.x,
        //                                 odom_gt.pose.pose.orientation.y,
        //                                 odom_gt.pose.pose.orientation.z,
        //                                 odom_gt.pose.pose.orientation.w
        //                                 );

        // qp_slam::planes backend_planes;
        backend_planes = qp_slam::planes();

        // backend_planes.odom = odom_;
        backend_planes.point_clouds = *input_cloud;
        // printf("{");
        front_end(pcl_cloud);
        // printf("},\n\n");
        printf("\n");
        cnt = 0;
        o_cnt++;

        topic_publish(*accumulated_cloud_x, x_all);
        topic_publish(*accumulated_cloud, all);
        accumulated_cloud_x = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        accumulated_cloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

        if(initialize) initialize = false;
        planes_pub.publish(backend_planes);
    }

    // 오도메트리 메시지 콜백 함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        
        // nav_msgs::Odometry odom_;
        
        odom_noise.pose.pose.position.x = msg->pose.pose.position.x;
        odom_noise.pose.pose.position.y = msg->pose.pose.position.y;
        odom_noise.pose.pose.position.z = msg->pose.pose.position.z;

        odom_noise.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        odom_noise.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        odom_noise.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        odom_noise.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    }   


    void odomCallback_gt(const nav_msgs::Odometry::ConstPtr& msg) {
        
        // nav_msgs::Odometry odom_;

        odom_gt.pose.pose.position.x = msg->pose.pose.position.x;
        odom_gt.pose.pose.position.y = msg->pose.pose.position.y;
        odom_gt.pose.pose.position.z = msg->pose.pose.position.z;

        odom_gt.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        odom_gt.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        odom_gt.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        odom_gt.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    }   

    void topic_publish(pcl::PointCloud<pcl::PointNormal>const input_cloud, ros::Publisher input_publish)
    {
        sensor_msgs::PointCloud2 pub_cloud;
    
        pcl::toROSMsg(input_cloud, pub_cloud); 
        pub_cloud.header.frame_id = "test"; 
        pub_cloud.header.stamp = ros::Time::now(); 
        input_publish.publish(pub_cloud); 

    }

    void topic_publish_xyz(pcl::PointCloud<pcl::PointXYZ>const input_cloud, ros::Publisher input_publish)
    {
        sensor_msgs::PointCloud2 pub_cloud;
    
        pcl::toROSMsg(input_cloud, pub_cloud); 
        pub_cloud.header.frame_id = "test"; 
        pub_cloud.header.stamp = ros::Time::now(); 
        input_publish.publish(pub_cloud); 

    }

    void front_end(pcl::PointCloud<pcl::PointXYZ> input_cloud)
    {
        sensor_msgs::PointCloud2 test_cloud;  
        pcl::toROSMsg(input_cloud, test_cloud);

        test_cloud.header.frame_id = "test";
        test_cloud.header.stamp = ros::Time::now();

        pub_2.publish(test_cloud);

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud ;
        filtered_cloud = filter_cloud(input_cloud, 0.3);

        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> extract_vector;
        
        extract_vector =  seg_plane(filtered_cloud.makeShared(),parameter_vector, 0.1, 5);

        topic_publish(*extract_vector[0], t_1);
        topic_publish(*extract_vector[1], t_2);
        topic_publish(*extract_vector[2], t_3);
        topic_publish(*extract_vector[3], t_4);
        topic_publish(*extract_vector[4], t_5);

        map_extracted_planes(extract_vector,x_planes,y_planes,hort_planes);
    }


    pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& extracted_cloud) {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(extracted_cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.1);
        ne.compute(*cloud_normals);

        return cloud_normals;
    }

    pcl::PointCloud<pcl::PointXYZ> filter_cloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, float filterRes)
    {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud.makeShared());
        sor.setLeafSize(filterRes, filterRes, filterRes);
        sor.filter(filtered_cloud);
        return filtered_cloud;
    }

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>  seg_plane(
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
    std::vector<uint16_t> maxIterations, 
    float distanceThreshold, 
    uint8_t planes_num)
{
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
    std::vector<uint8_t> plane_types;

    typename pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud = input_cloud; //초기 input cloud

    uint8_t plane_types_data;

    std::vector<Eigen::Vector3f> end_normal_vector;
    std::vector<float> end_distance; 
    uint8_t plane_num = 0;


    pcl::PointCloud<pcl::PointNormal>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointNormal>);
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> extracted_cloud_vec;


    for(int k = 0; k < planes_num; k++) {

        std::unordered_set<int> inliersResult;
        int largestPlaneSize = 0;
        Eigen::Vector3f bestNormal(0, 0, 0);
        float best_distance = 0;
        float real_best_distance = 0;
        uint8_t type;

        double in_out_ratio = 0; //outliner ratio
        
        for(int it = 0; it < maxIterations[k]; it++) {
        // for(int it = 0; it < 250; it++) {
            std::unordered_set<int> tempIndices; //인라이어의 index를 임시 저장하는 컨테이너
            while(tempIndices.size() < 3) {
                tempIndices.insert(rand() % remainingCloud->points.size());
            }

            auto iter = tempIndices.begin();
            pcl::PointXYZ point1 = remainingCloud->points[*iter]; ++iter;
            pcl::PointXYZ point2 = remainingCloud->points[*iter]; ++iter;
            pcl::PointXYZ point3 = remainingCloud->points[*iter];

            float a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
            float b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
            float c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
            float d = -(a * point1.x + b * point1.y + c * point1.z);

            for(int index = 0; index < remainingCloud->points.size(); index++) {
                if(tempIndices.find(index) == tempIndices.end()) {

                    pcl::PointXYZ point = remainingCloud->points[index];

                    float distance = fabs(a * point.x + b * point.y + c * point.z + d) / sqrt(a * a + b * b + c * c);
                    float real_distance = fabs(d) / sqrt(a * a + b * b + c * c);
                    
                    if(distance <= distanceThreshold) {
                        tempIndices.insert(index); // 평면에 속하는 pc의 index를 집어 넣음
                        best_distance = real_distance;
                    }
                }
            } // outlier의 point만 가지고 model 추출

            if(tempIndices.size() > largestPlaneSize) {
                largestPlaneSize = tempIndices.size();
                inliersResult = tempIndices;
                if(d < 0){a *= -1; b *= -1; c *= -1;}

                bestNormal = Eigen::Vector3f(a, b, c);  // 법선 벡터 업데이트
                real_best_distance = best_distance;
            } //평면이 계속 커지는 느낌

        } //plane 1개 get
      
        if(largestPlaneSize == 0) {
            break; // 더 이상 평면을 찾을 수 없음
        }
        
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

        for(int index = 0; index < remainingCloud->points.size(); index++) {
            pcl::PointXYZ point = remainingCloud->points[index];
            if(inliersResult.find(index) != inliersResult.end())
                planeCloud->points.push_back(point);
            else
                cloudOutliers->points.push_back(point); // input_data update ( outliners )
        }

        in_out_ratio = cloudOutliers->points.size()/double(remainingCloud->points.size());  
        parameter_vector[k] = ransac_number_calculate(in_out_ratio);  

        Eigen::Vector3f bestNormal_normalized = bestNormal.normalized();

        planes.push_back(planeCloud);
        remainingCloud = cloudOutliers;
        
        // if(best_distance < 0){bestNormal_normalized[0] = -bestNormal_normalized[0]; bestNormal_normalized[1] = -bestNormal_normalized[1]; bestNormal_normalized[2] = -bestNormal_normalized[2];}
        end_normal_vector.push_back(bestNormal_normalized); //normal _ vector 
        end_distance.push_back(real_best_distance); // plane_equation distance (D)

        pcl::PointCloud<pcl::PointNormal>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointNormal>);
        
        for (const auto& point : planeCloud->points) {
            pcl::PointNormal tmp_cloud;
            tmp_cloud.x = point.x;
            tmp_cloud.y = point.y;
            tmp_cloud.z = point.z;
            tmp_cloud.normal_x = bestNormal_normalized[0];
            tmp_cloud.normal_y = bestNormal_normalized[1];
            tmp_cloud.normal_z = bestNormal_normalized[2];
            tmp_cloud.curvature = real_best_distance;

            extracted_cloud->points.push_back(tmp_cloud);
        }

            extracted_cloud_vec.push_back(extracted_cloud);
  
    }

    for (int i = 0; i < 4; i++) {
        if (i < extracted_cloud_vec.size() && !extracted_cloud_vec[i]->points.empty()) {
            pcl::PointNormal first_point = extracted_cloud_vec[i]->points.front();
            //ROS_INFO("%d   %f %f %f", i, first_point.normal_x, first_point.normal_y, first_point.normal_z);
        }
    }

    return extracted_cloud_vec;
}

int ransac_number_calculate(double in_out_ratio)
{ 
    int T = std::log(1 - 0.99) / std::log(1 - std::pow(1 - in_out_ratio, 3));      

    if(T > max_time) return max_time;
          
    return T;
} // ransac parmeter calculate

    void map_extracted_planes(const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& extracted_cloud_vec,     
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes)
    {
        int test =0;
        for (const auto& cloud_seg_body : extracted_cloud_vec){
            if (cloud_seg_body->points.size() < 50) continue; //extracted_cloud_vec points 갯수가 1000개를 못넘으면 plane_변환하지 않음

            Eigen::Vector4d plane_params(cloud_seg_body->back().normal_x,
                                        cloud_seg_body->back().normal_y,
                                        cloud_seg_body->back().normal_z,
                                        cloud_seg_body->back().curvature);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_body_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud_seg_body, *cloud_seg_body_points);

            // Eigen::Vector4d를 gtsam::Vector4로 변환
            gtsam::Vector4 plane_normal(plane_params(0), plane_params(1), plane_params(2), plane_params(3)); //저장벡터 

            int plane_type = add_planes_to_graph(plane_normal, cloud_seg_body_points, x_vert_planes, y_vert_planes, hort_planes);
           test ++;

        }

        ROS_INFO("x_size: %d,y_size: %d,z_size: %d", x_vert_planes.size(),y_vert_planes.size(),hort_planes.size());


    }

    sensor_msgs::PointCloud2 cloud_topic( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        sensor_msgs::PointCloud2 test_cloud;  
        pcl::toROSMsg(*input_cloud, test_cloud);

        test_cloud.header.frame_id = "test";
        test_cloud.header.stamp = ros::Time::now();
        
        return test_cloud;
    }


    int add_planes_to_graph(gtsam::Vector4 plane_normal,
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_seg_body,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes
    )
    {
        int plane_id;
        int plane_type = -1;   

        gtsam::Vector4 det_plane_map_frame = convert_plane_to_map_frame(plane_normal);

        // ROS_INFO("local normal : %f %f %f %f", plane_normal(0), plane_normal(1), plane_normal(2), plane_normal(3));
        if (fabs(det_plane_map_frame(0)) > fabs(det_plane_map_frame(1)) &&
            fabs(det_plane_map_frame(0)) > fabs(det_plane_map_frame(2)))
            plane_type = X_VERT_PLANE; //x 축 평면

        else if (fabs(det_plane_map_frame(1)) > fabs(det_plane_map_frame(0)) &&
                fabs(det_plane_map_frame(1)) > fabs(det_plane_map_frame(2)))
            {
                plane_type = Y_VERT_PLANE; //y 축 평면
                // cnt ++;
            }
        else if (fabs(det_plane_map_frame(2)) > fabs(det_plane_map_frame(0)) &&
                fabs(det_plane_map_frame(2)) > fabs(det_plane_map_frame(1)))
            plane_type = HORT_PLANE; //z 축 평면
            
            // local pointcloud Y - WALL

        plane_id = factor_planes(plane_type, 
                                 cloud_seg_body,
                                 det_plane_map_frame, 
                                 plane_normal,
                                 x_vert_planes, 
                                 y_vert_planes, 
                                 hort_planes);

        return plane_type; 
    }

    int factor_planes(int plane_type,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_seg_body,
                    gtsam::Vector4 det_plane_map_frame, // global plane normal_vector & distance
                    gtsam::Vector4 det_plane_body_frame, // local plane normal_vector & distance
                    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                    std::unordered_map<int, HorizontalPlanes>& hort_planes)
    {
        int data_association = -1;

        // associate_plane 함수 호출
        data_association = associate_plane(plane_type,
                                        det_plane_body_frame,
                                        cloud_seg_body,
                                        x_vert_planes,
                                        y_vert_planes,
                                        hort_planes);

        switch (plane_type) {
           case X_VERT_PLANE: { 
                if (x_vert_planes.empty() || data_association == -1) {

                    data_association = x_vert_planes.size() + 1;

                    VerticalPlanes vert_plane;
                    vert_plane.id = data_association;
                    vert_plane.plane = det_plane_map_frame; //global 기준 coefficent
                    vert_plane.cloud_seg_body = cloud_seg_body; 
                    vert_plane.cloud_seg_body_vec.push_back(cloud_seg_body); 
                    vert_plane.plane_node = det_plane_body_frame; //local 기준 coefficent 
                    
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<pcl::PointNormal>());
                    vert_plane.cloud_seg_map = cloud_seg_detected; // cloud_seg_map를 초기화합니다.

                    Eigen::Isometry3d current_keyframe_pose = odom_get(odom_);

                    for (int j = 0; j < cloud_seg_body->points.size(); ++j) {
                        pcl::PointNormal dst_pt;
                        dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(cloud_seg_body->points[j].x, cloud_seg_body->points[j].y, cloud_seg_body->points[j].z, 1.0)).cast<float>();
                        vert_plane.cloud_seg_map->points.push_back(dst_pt);
                    }

                    vert_plane.covariance = Eigen::Matrix3d::Identity();

                    x_vert_planes.insert({vert_plane.id, vert_plane});

                    if(initialize)
                    {
                        // ROS_INFO("%f %f %f %f",x_vert_planes.at(data_association).plane(0),
                        //                         x_vert_planes.at(data_association).plane(1),
                        //                         x_vert_planes.at(data_association).plane(2),
                        //                         x_vert_planes.at(data_association).plane(3)
                        //                         );

                        // ROS_INFO("%f %f %f %f",det_plane_body_frame(0),
                        //                         det_plane_body_frame(1),
                        //                         det_plane_body_frame(2),
                        //                         det_plane_body_frame(3)
                        //                         );

                        qp_slam::plane backend_plane;

                        backend_plane.id = data_association;
                        backend_plane.nx = x_vert_planes.at(data_association).plane(0);
                        backend_plane.ny = x_vert_planes.at(data_association).plane(1);
                        backend_plane.nz = x_vert_planes.at(data_association).plane(2);
                        backend_plane.d  = x_vert_planes.at(data_association).plane(3);

                        backend_plane.nx_local = det_plane_body_frame(0);
                        backend_plane.ny_local = det_plane_body_frame(1);
                        backend_plane.nz_local = det_plane_body_frame(2);
                        backend_plane.d_local  = det_plane_body_frame(3);

                        backend_planes.all_planes.push_back(backend_plane);
                    }

                }
                break;
            }

            case Y_VERT_PLANE: { 
                if (y_vert_planes.empty() || data_association == -1) {
                    // ROS_INFO("ZLR");
                    data_association = y_vert_planes.size() + 1;

                    VerticalPlanes vert_plane;
                    vert_plane.id = data_association;
                    vert_plane.plane = det_plane_map_frame;
                    vert_plane.cloud_seg_body = cloud_seg_body;
                    vert_plane.cloud_seg_body_vec.push_back(cloud_seg_body);
                    vert_plane.plane_node = det_plane_body_frame;

                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<pcl::PointNormal>());
                    vert_plane.cloud_seg_map = cloud_seg_detected; // cloud_seg_map를 초기화합니다.

                    Eigen::Isometry3d current_keyframe_pose = odom_get(odom_);

                    for (int j = 0; j < cloud_seg_body->points.size(); ++j) {
                        pcl::PointNormal dst_pt;
                        dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(cloud_seg_body->points[j].x, cloud_seg_body->points[j].y, cloud_seg_body->points[j].z, 1.0)).cast<float>();
                        vert_plane.cloud_seg_map->points.push_back(dst_pt);
                    }
                    
                    vert_plane.covariance = Eigen::Matrix3d::Identity();

                    y_vert_planes.insert({vert_plane.id, vert_plane});

                    if(initialize)
                    {

                        qp_slam::plane backend_plane;

                        backend_plane.id = data_association;
                        backend_plane.nx = y_vert_planes.at(data_association).plane(0);
                        backend_plane.ny = y_vert_planes.at(data_association).plane(1);
                        backend_plane.nz = y_vert_planes.at(data_association).plane(2);
                        backend_plane.d  = y_vert_planes.at(data_association).plane(3);

                        backend_plane.nx_local = det_plane_body_frame(0);
                        backend_plane.ny_local = det_plane_body_frame(1);
                        backend_plane.nz_local = det_plane_body_frame(2);
                        backend_plane.d_local  = det_plane_body_frame(3);
                    
                        backend_planes.all_planes.push_back(backend_plane);
                    }

                }
                break;
            }
            


            default:
                break;

        }
        
            backend_planes.odom = odom_gt;
            backend_planes.noise_odom = odom_noise;

        // ROS_INFO("x_size: %d,y_size: %d,z_size: %d", x_vert_planes.size(),y_vert_planes.size(),hort_planes.size());
        // convert_plane_points_to_map(x_vert_planes, y_vert_planes, hort_planes);
        return data_association; // data_association 값 반환
    }

    void convert_plane_points_to_map(
        std::unordered_map<int, VerticalPlanes>& x_vert_planes,
        std::unordered_map<int, VerticalPlanes>& y_vert_planes,
        std::unordered_map<int, HorizontalPlanes>& hort_planes){
 
        Eigen::Isometry3d current_keyframe_pose = odom_get(odom_);

        for (auto& x_vert_plane : x_vert_planes) {

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<pcl::PointNormal>());

            for (int j = 0; j < x_vert_plane.second.cloud_seg_body->points.size(); ++j) {
                    pcl::PointNormal dst_pt;
                dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(x_vert_plane.second.cloud_seg_body->points[j].x, 
                                                                                    x_vert_plane.second.cloud_seg_body->points[j].y, 
                                                                                    x_vert_plane.second.cloud_seg_body->points[j].z, 
                                                                                    1.0)).cast<float>();
                cloud_seg_map->points.push_back(dst_pt);
            }
            x_vert_plane.second.cloud_seg_map = cloud_seg_map; //2번째 plane에 저장
            
        }

        for (auto& y_vert_plane : y_vert_planes) {

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<pcl::PointNormal>());

            for (int j = 0; j < y_vert_plane.second.cloud_seg_body->points.size(); ++j) {
                    pcl::PointNormal dst_pt;
                dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(y_vert_plane.second.cloud_seg_body->points[j].x, 
                                                                                    y_vert_plane.second.cloud_seg_body->points[j].y, 
                                                                                    y_vert_plane.second.cloud_seg_body->points[j].z, 
                                                                                    1.0)).cast<float>();
                cloud_seg_map->points.push_back(dst_pt);
            }
            y_vert_plane.second.cloud_seg_map = cloud_seg_map;
            
        }

    }

  int associate_plane(
    const int& plane_type,
    const gtsam::Vector4& det_plane,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_seg_body,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::unordered_map<int, HorizontalPlanes>& hort_planes) 
    {
    int data_association = -1;
    double vert_min_maha_dist = 100;
    double hort_min_maha_dist = 100;

    Eigen::Isometry3d m2n = odom_get(odom_); // map2node 전역->지역 좌표계로 변환  
    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
    transform_matrix.block<3, 3>(0, 0) = m2n.rotation().inverse();

    gtsam::Vector4 normal_plane(det_plane(0), det_plane(1), det_plane(2), det_plane(3));

    qp_slam::plane backend_plane;
    
    switch (plane_type) {
    case X_VERT_PLANE: {
            int cnt_tt = 0;  
            for (const auto& x_vert_plane : x_vert_planes) { 

                gtsam::Vector4 local_plane = transform_matrix * x_vert_plane.second.plane; // local 좌표계에서 plane, x-vert_plane은 전역좌표계에서 저장되는 x축 벽인듯함
                Eigen::Vector3d error = local_plane.head(3) - det_plane.head(3);

                double maha_dist = sqrt(error.transpose() * x_vert_plane.second.covariance.inverse() * error);         

                if (std::isnan(maha_dist) || maha_dist < 1e-3) {
                    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                    maha_dist = sqrt(error.transpose() * cov * error);
                }

                if (maha_dist < 0.5) {
                    vert_min_maha_dist = maha_dist;
                    data_association = x_vert_plane.second.id;
                }
                else continue;

          if (maha_dist < 0.3) { 
            
                if (!x_vert_planes.at(data_association).cloud_seg_map->empty()) {
                    float min_segment = std::numeric_limits<float>::max();

                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<pcl::PointNormal>());
                    Eigen::Isometry3d current_keyframe_pose = odom_get(odom_);

                     for (int j = 0; j < cloud_seg_body->points.size(); ++j) {
                         pcl::PointNormal dst_pt;
                         dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(cloud_seg_body->points[j].x, cloud_seg_body->points[j].y, cloud_seg_body->points[j].z, 1.0)).cast<float>();
                         cloud_seg_detected->points.push_back(dst_pt);
                     }

                     bool valid_neighbour = check_point_neighbours(
                         x_vert_planes.at(data_association).cloud_seg_map, cloud_seg_detected);
                    // ROS_INFO("----------------------dong %d ass: %d-----------------",valid_neighbour,data_association);
                     if (!valid_neighbour) {
                         data_association = -1;
                     }

                     else{

                            backend_plane.id = data_association;
                            backend_plane.nx = x_vert_planes.at(data_association).plane(0);
                            backend_plane.ny = x_vert_planes.at(data_association).plane(1);
                            backend_plane.nz = x_vert_planes.at(data_association).plane(2);
                            backend_plane.d  = x_vert_planes.at(data_association).plane(3);

                            backend_plane.nx_local = normal_plane(0);
                            backend_plane.ny_local = normal_plane(1);
                            backend_plane.nz_local = normal_plane(2);
                            backend_plane.d_local  = normal_plane(3);

                            // *accumulated_cloud_x += *(cloud_seg_detected); //global pcd
                            *accumulated_cloud_x += *x_vert_planes.at(data_association).cloud_seg_map; //local pcd
                            
                            sensor_msgs::PointCloud2 convert_cloud;
                            pcl::toROSMsg(*x_vert_planes.at(data_association).cloud_seg_map, convert_cloud);

                            backend_planes.all_planes.push_back(backend_plane);

                            // topic_publish(*accumulated_cloud_x, x_all);
                            break;
                     }
                }
                
            } 


            else {
                data_association = -1;
            }

            }


            break;
        }


        case Y_VERT_PLANE: {
            int cnt_tt = 0;
                            ROS_INFO("prior_y %f,%f,%f,%f",
                                                normal_plane(0),
                                                normal_plane(1),
                                                normal_plane(2),
                                                normal_plane(3)
                                                );   
            for (const auto& y_vert_plane : y_vert_planes) { 
                // ROS_INFO("dong %d",y_vert_planes.size());
                
                gtsam::Vector4 local_plane = transform_matrix * y_vert_plane.second.plane; // local 좌표계에서 plane, x-vert_plane은 전역좌표계에서 저장되는 x축 벽인듯함

                Eigen::Vector3d error = local_plane.head(3) - det_plane.head(3);

                double maha_dist = sqrt(error.transpose() * y_vert_plane.second.covariance.inverse() * error);         

            
                if (std::isnan(maha_dist) || maha_dist < 1e-3) {
                    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                    maha_dist = sqrt(error.transpose() * cov * error);
                }

                                       ROS_INFO("after_y %f,%f,%f,%f\t %d\t%f",
                                                local_plane(0),
                                                local_plane(1),
                                                local_plane(2),
                                                local_plane(3),
                                                y_vert_plane.second.id,
                                                maha_dist
                                                );      

                if (maha_dist < 0.5) {
                    vert_min_maha_dist = maha_dist;
                    data_association = y_vert_plane.second.id;

                }
                else continue;
          


            if (maha_dist < 0.3) {       
                if (!y_vert_planes.at(data_association).cloud_seg_map->empty()) {
                    float min_segment = std::numeric_limits<float>::max();

                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<pcl::PointNormal>());
                    Eigen::Isometry3d current_keyframe_pose = odom_get(odom_);

                     for (int j = 0; j < cloud_seg_body->points.size(); ++j) {
                         pcl::PointNormal dst_pt;
                         dst_pt.getVector4fMap() = (current_keyframe_pose * Eigen::Vector4d(cloud_seg_body->points[j].x, cloud_seg_body->points[j].y, cloud_seg_body->points[j].z, 1.0)).cast<float>();
                         cloud_seg_detected->points.push_back(dst_pt);
                     }

                     bool valid_neighbour = check_point_neighbours(
                         y_vert_planes.at(data_association).cloud_seg_map, cloud_seg_detected);


                     if (!valid_neighbour) {
                         data_association = -1;
                     }
                     else{
                            // *accumulated_cloud += *(cloud_seg_detected);
                    ROS_INFO("----------------------dong %d ass: %d-----------------",valid_neighbour,data_association);
                            *accumulated_cloud += *y_vert_planes.at(data_association).cloud_seg_map;

                            backend_plane.id = data_association;
                            backend_plane.nx = y_vert_planes.at(data_association).plane(0);
                            backend_plane.ny = y_vert_planes.at(data_association).plane(1);
                            backend_plane.nz = y_vert_planes.at(data_association).plane(2);
                            backend_plane.d  = y_vert_planes.at(data_association).plane(3);

                            backend_plane.nx_local = normal_plane(0);
                            backend_plane.ny_local = normal_plane(1);
                            backend_plane.nz_local = normal_plane(2);
                            backend_plane.d_local  = normal_plane(3);

                            sensor_msgs::PointCloud2 convert_cloud;
                            // pcl::toROSMsg(*y_vert_planes.at(data_asso0ciation).cloud_seg_map, convert_cloud);
                            backend_plane.pointclouds = convert_cloud;

                            backend_planes.all_planes.push_back(backend_plane);

                            break;
                            // topic_publish(*accumulated_cloud, all);
                     }

                }
                
            } 

            else {
                // ROS_INFO("----------------------dong %d error-----------------",data_association);
                data_association = -1;
            }

            }



            break;
        }

        default:
            break;
    }

    return data_association;
}

    bool check_point_neighbours(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_1, //plane 기준에 저장되어 있는 global pointcloud
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_2, //현재 global pointcloud
        float min_dist_squared = 0.5, // 예: 0.5m 거리의 제곱
        int min_neighbour_count = 100) {

        int point_count = 0;

        int point_count_1 = cloud_1->points.size()*0.1;

        for(int i = 0; i < cloud_1->points.size(); i++)
        {
            for(int j = 0; j < cloud_2->points.size(); j++)
            {
               float dist = (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap()).squaredNorm();
                if (dist < min_dist_squared) {
                    point_count++;
                    break;
                }           
            }
            if (point_count > point_count_1) {
                return true;
            }
        }

        return false; // 충분한 이웃을 찾지 못함
    }

    gtsam::Vector4 convert_plane_to_map_frame(gtsam::Vector4 det_plane_body_frame) //local2global to normal vector
    {      
        Eigen::Isometry3d w2n = odom_get(odom_);
        Eigen::Vector3d body_frame(det_plane_body_frame(0), det_plane_body_frame(1), det_plane_body_frame(2));
        // 법선 벡터 변환
        Eigen::Vector3d rotated_normal = w2n.rotation() * body_frame; //rotation => robot의 회전에 대한 정보, 0~2는 normal vector인걸 global로 변환한것

        double d_global = det_plane_body_frame(3) - w2n.translation().dot(rotated_normal);

        gtsam::Vector4 det_plane_map_frame(rotated_normal(0), rotated_normal(1), rotated_normal(2), d_global);

        return det_plane_map_frame;
    } //g20는 Eigen과 연관이 매우 높은데 gtsam은 eigen과 연관이 없어서 일반적으로 gtsam->eigen, eigen->gtsam 과정을 거쳐야함


    Eigen::Isometry3d odom_get(nav_msgs::Odometry odom)
    {
        Eigen::Isometry3d w2n_pose = Eigen::Isometry3d::Identity();

        Eigen::Vector3d translation(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                odom.pose.pose.position.z);

        Eigen::Quaterniond quat(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z
        );

        Eigen::Matrix3d rotation = quat.toRotationMatrix();
        
        w2n_pose.translate(translation);
        w2n_pose.rotate(rotation);

        return w2n_pose;
    } //기존 s-graph는 keyframe으로 저장해서 했는데 일단 real-time으로 바꾸어봄


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "front_plane");

    graph graph;

    ros::spin();

    return 0;
}
