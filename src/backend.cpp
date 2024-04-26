#include "qp_slam/backend.h"
#include <ros/ros.h>


backend::backend(ros::NodeHandle nh) {
    get_params(nh);

    plane_sub = nh.subscribe<qp_slam::planes>(plane_topic_, 1, &backend::plane_callback, this);
    key_sub = nh.subscribe<qp_slam::pose>(key_topic, 1, &backend::pose_callback, this);
}

void backend::get_params(ros::NodeHandle nh)
{

    nh.param<string>("plane_topic", plane_topic_, "/plane");
    nh.param<string>("key_topic", key_topic, "/key_");

    points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/noise_points", 1);
    // points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/platform/odometry", 1);
    points_pub_2 = nh.advertise<sensor_msgs::PointCloud2>("/optimize_points", 1);
    points_pub_3 = nh.advertise<sensor_msgs::PointCloud2>("/real", 1);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/graph_odom", 1);

    key_pub = nh.advertise<qp_slam::key_m>("/key_pose", 1);
//--------------------noise_model start----------------------//
    priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());

    odometryNoise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 2.5, 2.5, 2.5, 2.5, 2.5, 2.5).finished()); 

    x_p_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2,0.2,0.2));
//--------------------noise_model end----------------------//


}

void backend::pose_callback(const qp_slam::pose::ConstPtr& msg)
{

//--------------------odom_get start----------------------//
    odom_id ++;
    gtsam::Rot3 quaternion_noise(msg->pose_odom.pose.pose.orientation.w, 
                                0,
                                0, 
                                msg->pose_odom.pose.pose.orientation.z);

    gtsam::Point3 position_noise(msg->pose_odom.pose.pose.position.x, 
                                msg->pose_odom.pose.pose.position.y, 
                                msg->pose_odom.pose.pose.position.z);

    gtsam::Vector3 euler_angles_noise = quaternion_noise.ypr();

    // Extract roll, pitch, and yaw
    float roll_n = euler_angles_noise[2];
    float pitch_n = euler_angles_noise[1];
    float yaw_n = euler_angles_noise[0];

    gtsam::Pose3 current_pose_noise(gtsam::Rot3::Ypr(yaw_n,pitch_n,roll_n), position_noise);
//noise_odom (실제 pose)
//--------------------odom_get end----------------------//

//--------------------pose_factor start ----------------------//

    if(odom_id == 0) 
    {
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(odom_id), current_pose_noise, priorNoise);
        initialEstimate.insert(X(odom_id), current_pose_noise);
    }

    else 
    {
        gtsam::Pose3 odomChange = pre_pose.between(current_pose_noise);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(odom_id-1), X(odom_id), odomChange, odometryNoise);
        initialEstimate.insert(X(odom_id), current_pose_noise);
    }

     pre_pose = gtsam::Pose3(gtsam::Rot3::Ypr(yaw_n, pitch_n, roll_n), position_noise);   


//--------------------pose_factor end ----------------------//


    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(100); 
    params.setRelativeErrorTol(1e-5); 
    params.setAbsoluteErrorTol(1e-5);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
    gtsam::Values result = optimizer.optimize();
    
//****************************pointcloud pub****************************************//

    // 최적화 결과에서 포즈 추출

    pcl::PointCloud<pcl::PointXYZ> all_pointclouds_1;
    map_cloud.push_back(msg->pose_point_clouds); 

    for(int i = 0; i <= odom_id; i++)
    {
        gtsam::Pose3 finalPose = result.at<gtsam::Pose3>(X(i));

        // 이동 추출
        Eigen::Vector3d translation_1(finalPose.translation().x(),
                                    finalPose.translation().y(), 
                                    0);

        // 회전을 쿼터니언으로 변환
        gtsam::Rot3 rot_1 = finalPose.rotation();
        Eigen::Quaterniond quat_1 = Eigen::Quaterniond(rot_1.toQuaternion());

        Eigen::Vector3d translation_convert_1( translation_1(0),translation_1(1),translation_1(2));

        Eigen::Quaterniond quat_convert_1(
            quat_1.w(),
            0,
            0,
            quat_1.z()
        );


        if(i == odom_id) //마지막 최적화
        {

//--------------------odom_pub start ----------------------//

            nav_msgs::Odometry graph_odom;
            graph_odom.header.stamp = ros::Time::now();
            graph_odom.header.frame_id = "test"; // or your global frame id
            graph_odom.child_frame_id = "graph_odom"; // or your robot frame id

            // Set position
            graph_odom.pose.pose.position.x = translation_convert_1(0);
            graph_odom.pose.pose.position.y = translation_convert_1(1);
            graph_odom.pose.pose.position.z = 0;

            // Set orientation
            graph_odom.pose.pose.orientation.w = quat_convert_1.w();
            graph_odom.pose.pose.orientation.x = 0;
            graph_odom.pose.pose.orientation.y = 0;
            graph_odom.pose.pose.orientation.z = quat_convert_1.z();

            // Publish the odometry message
            odom_pub.publish(graph_odom);

            // Additionally, to broadcast a transform using tf
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "test";
            transformStamped.child_frame_id = "graph_odom";
            transformStamped.transform.translation.x = translation_convert_1(0);
            transformStamped.transform.translation.y = translation_convert_1(1);
            transformStamped.transform.translation.z = 0;
            transformStamped.transform.rotation.w = quat_convert_1.w();
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = quat_convert_1.z();

            // Broadcast the transform
            br.sendTransform(transformStamped);

            qp_slam::key_m key_point;
            key_point.key_odom = graph_odom;
            key_point.key_point_clouds = msg->pose_point_clouds;
            key_pub.publish(key_point);

            // pose, image_raw

//--------------------odom_pub end ----------------------//
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ> input_cloud;
            pcl::fromROSMsg(map_cloud[i], input_cloud);

            Eigen::Matrix3d rotation_1 = quat_convert_1.toRotationMatrix();
            Eigen::Isometry3d w2n_pose_1 = Eigen::Isometry3d::Identity();

            w2n_pose_1.translate(translation_convert_1);
            w2n_pose_1.rotate(rotation_1);
            
            // ROS_INFO("estim_Quaternion: w = %f, x = %f, y = %f, z = %f", quat_convert_1.w(), quat_convert_1.x(), quat_convert_1.y(), quat_convert_1.z());
            // ROS_INFO("estim_Translation: x = %f, y = %f, z = %f", translation_convert_1(0), translation_convert_1(1), translation_convert_1(2));

            Eigen::Matrix4f transform_1 = w2n_pose_1.matrix().cast<float>();

            pcl::PointCloud<pcl::PointXYZ> transformed_cloud_1;
            pcl::transformPointCloud(input_cloud, transformed_cloud_1, transform_1);
            
            all_pointclouds_1 += transformed_cloud_1;
        }
    }

    pcl::PCLPointCloud2 temp_cloud;
    pcl::toPCLPointCloud2(all_pointclouds_1, temp_cloud);

    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::fromPCL(temp_cloud, output_cloud);

    auto output_cloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>(output_cloud);

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud_3 = filter_cloud(output_cloud_ptr, 0.3); 

    sensor_msgs::PointCloud2 pub_cloud_1;
    
    // 변환 함수 사용 시 네임스페이스 수정
    pcl::toROSMsg(filtered_cloud_3, pub_cloud_1);
    pub_cloud_1.header.frame_id = "test"; 
    pub_cloud_1.header.stamp = ros::Time::now(); 
    points_pub_2.publish(pub_cloud_1);

    result.print("Final LM Result:\n");

}

void backend::plane_callback(const qp_slam::planes::ConstPtr& msg)
{
    ROS_INFO_STREAM("-------------------------------------------------\n");       
    pcl::PointCloud<pcl::PointXYZ> current_pointclouds;


//--------------------plane_set_assign start ----------------------//
    for(size_t n = 0; n < msg->all_planes.size(); ++n) {
        const auto& plane = msg->all_planes[n];
        gtsam::OrientedPlane3 new_plane(plane.nx, plane.ny, plane.nz, plane.d);

        bool isDuplicate = false;
        for(const auto& plane_ : plane_set) {
            gtsam::Point3 plane_normal = plane_.normal().point3();
            gtsam::Point3 new_plane_normal = new_plane.normal().point3();
            
            if(plane_normal.x() == new_plane_normal.x() &&
            plane_normal.y() == new_plane_normal.y() &&
            plane_normal.z() == new_plane_normal.z() &&
            plane_.distance() == new_plane.distance()) {

                isDuplicate = true;
                break;
            }
        }
        if(!isDuplicate) {
                plane_set.push_back(new_plane);

                // ROS_INFO("Added plane: %f, %f, %f, %f", 
                //         new_plane.normal().point3().x(), new_plane.normal().point3().y(), 
                //         new_plane.normal().point3().z(), new_plane.distance());
            }
    }

//--------------------plane_set_assign end ----------------------//


//--------------------plane_factor start ----------------------//
    for(int k = 0; k < msg->all_planes.size(); k++)
    {
        const auto& plane_compare = msg->all_planes[k];
        gtsam::OrientedPlane3 new_plane_compare(plane_compare.nx, plane_compare.ny, 
                                        plane_compare.nz, plane_compare.d);

        gtsam::OrientedPlane3 plane_measurment(plane_compare.nx_local, plane_compare.ny_local, 
                                        plane_compare.nz_local, plane_compare.d_local); //measurment_plane


        for(int j = 0; j < plane_set.size(); j ++)
        {

            gtsam::Point3 plane_normal = plane_set[j].normal().point3();
            gtsam::Point3 new_plane_normal_compare =  new_plane_compare.normal().point3();
            
            if(new_plane_normal_compare.x() == plane_normal.x() &&
            new_plane_normal_compare.y() == plane_normal.y() &&
            new_plane_normal_compare.z() == plane_normal.z() &&
            new_plane_compare.distance() == plane_set[j].distance())
            {
                gtsam::Vector4 z = plane_measurment.planeCoefficients();

                graph.emplace_shared<gtsam::OrientedPlane3Factor>(z,x_p_noise,X(odom_id),P(j));
                
                if(!initialEstimate.exists(P(j))){
                    initialEstimate.insert(P(j),new_plane_compare);
                }

            }
        }
    }

//--------------------plane_factor end ----------------------//

//****************************pointcloud pub****************************************//


/////////////////////////////////////////////////////
    sensor_msgs::PointCloud2ConstPtr cloud_copy = boost::make_shared<sensor_msgs::PointCloud2>(msg->point_clouds);

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud_ = filter_cloud(cloud_copy, 0.3); 

    Eigen::Vector3d translation(msg->noise_odom.pose.pose.position.x,
                            msg->noise_odom.pose.pose.position.y,
                            msg->noise_odom.pose.pose.position.z);

    Eigen::Quaterniond quat(
        msg->noise_odom.pose.pose.orientation.w,
        0,
        0,
        msg->noise_odom.pose.pose.orientation.z
    );

    Eigen::Matrix3d rotation = quat.toRotationMatrix();
    Eigen::Isometry3d w2n_pose = Eigen::Isometry3d::Identity();

    w2n_pose.translate(translation);
    w2n_pose.rotate(rotation);

    Eigen::Matrix4f transform = w2n_pose.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(filtered_cloud_, transformed_cloud, transform);

    // Now, you can append the transformed cloud to your cumulative cloud.
    all_pointclouds += transformed_cloud;
            
    sensor_msgs::PointCloud2 pub_cloud;
    
    pcl::toROSMsg(all_pointclouds, pub_cloud); 
    pub_cloud.header.frame_id = "test"; 
    pub_cloud.header.stamp = ros::Time::now(); 
    points_pub_.publish(pub_cloud); 

//****************************pointcloud pub****************************************//

//****************************pointcloud pub****************************************//

    Eigen::Vector3d translation_3(msg->odom.pose.pose.position.x,
                            msg->odom.pose.pose.position.y,
                            msg->odom.pose.pose.position.z);

    Eigen::Quaterniond quat_3(
        msg->odom.pose.pose.orientation.w,
        0,
        0,
        msg->odom.pose.pose.orientation.z
    );

    Eigen::Matrix3d rotation_3 = quat_3.toRotationMatrix();
    Eigen::Isometry3d w2n_pose_3 = Eigen::Isometry3d::Identity();

    w2n_pose_3.translate(translation_3);
    w2n_pose_3.rotate(rotation_3);

    Eigen::Matrix4f transform_3 = w2n_pose_3.matrix().cast<float>();

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud_3;
    pcl::transformPointCloud(filtered_cloud_, transformed_cloud_3, transform_3);

    // Now, you can append the transformed cloud to your cumulative cloud.
    all_pointclouds_3 += transformed_cloud_3;
            
    sensor_msgs::PointCloud2 pub_cloud_3;
    
    pcl::toROSMsg(all_pointclouds_3, pub_cloud_3);
    pub_cloud_3.header.frame_id = "test";
    pub_cloud_3.header.stamp = ros::Time::now();
    points_pub_3.publish(pub_cloud_3);

    

    // terminal_view(msg);


    ROS_INFO("END\n\n");

    // odom_id ++;
}

void backend::terminal_view(const qp_slam::planes::ConstPtr& msg)
{
    ROS_INFO_STREAM("----------------------------------");
    const nav_msgs::Odometry& odom = msg->odom;
    ROS_INFO_STREAM("Odometry: Position: (" << odom.pose.pose.position.x << ", "
                    << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z
                    << "), Orientation: (" << odom.pose.pose.orientation.x << ", "
                    << odom.pose.pose.orientation.y << ", "
                    << odom.pose.pose.orientation.z << ", "
                    << odom.pose.pose.orientation.w << ")");

    // for(const auto& plane : msg->all_planes)
    // {
    //     ROS_INFO_STREAM("Plane: ID=" << plane.id 
    //                     << ", Normal: (" << plane.nx << ", "
    //                     << plane.ny << ", " << plane.nz << ")"
    //                     << ", Distance: " << plane.d);

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(plane.pointclouds, *cloud);

    //     // ROS_INFO_STREAM("Plane PointCloud: Total Points=" << cloud->points.size());
    // }

    // ROS_INFO("Total Planes: %d", static_cast<int>(msg->all_planes.size()));
    ROS_INFO_STREAM("----------------------------------");
}

void backend::visualizeResults(const gtsam::Values& results) {
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;  // 고유 ID 할당을 위한 변수

    for (const auto& key_value : results) {
        // 각 키에 대한 값을 처리합니다.
        auto key = key_value.key;
        auto value = dynamic_cast<const gtsam::Pose3*>(&key_value.value);
        if (value) {
            // Pose3에 대한 처리: 오도메트리 및 평면의 포즈를 시각화합니다.
            visualization_msgs::Marker marker;
            // marker 특성 설정 (위치, 방향, 타입 등)
            marker.header.frame_id = "world";  // 적절한 프레임 ID 설정
            marker.header.stamp = ros::Time::now();
            marker.ns = "result_poses";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::ARROW;  // 또는 CUBE 등
            // marker.pose 및 기타 속성 설정
            marker_array.markers.push_back(marker);
        }
        // 평면이나 다른 객체에 대한 처리 추가 가능
    }

    marker_pub_.publish(marker_array);  // 시각화 마커 배열 발행
}

pcl::PointCloud<pcl::PointXYZ> backend::filter_cloud(const sensor_msgs::PointCloud2ConstPtr& input, float filterRes)
{
    pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*input, *cloud);
    
    // pcl 의 VoxelGrid 타입
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(cloud_filtered);


    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    pcl::fromPCLPointCloud2(cloud_filtered, final_cloud);

    return final_cloud;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "backend_node");
    ros::NodeHandle nh;

    backend backend(nh);
    ros::spin();

    return 0;
}
