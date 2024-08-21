#include "orb_slam3_ros/node.h"
#include "orb_slam3_ros/boost_serialization.h"
#include "Thirdparty/Sophus/sophus/se3.hpp"

#include <ros/package.h> // to get the package path

namespace orb_slam3_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM3::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type), tfListener_(tfBuffer_),
        slam_initialized_(false), scale_factor_(1.0), min_observations_per_point_(2) {
    node_name_ = ros::this_node::getName();
    namespace_ = ros::this_node::getNamespace();

    // initial tf
    latest_local_tf_ = tf2::Transform(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));
    initialize_node();
}

node::~node() {
    ROS_INFO("[ORB_SLAM3_ROS] Terminating ORB-SLAM3 node.");
    orb_slam_->Shutdown();
    delete orb_slam_;
}

void node::initialize_node() {
    initialize_ros_pre_slam();
    initialize_orb_slam3();
    initialize_ros_post_slam();
}

void node::initialize_ros_pre_slam() {
    // Publish flags
    node_handle_.param(node_name_+"/publish_map", publish_map_, false);
    node_handle_.param(node_name_+"/publish_rendered_image", publish_rendered_image_, false);
    node_handle_.param(node_name_+"/publish_pose", publish_pose_, false);
    node_handle_.param(node_name_+"/publish_tf", publish_tf_, false);
    if (publish_tf_ && !publish_pose_) { // you simply don't publish tf without pose
        publish_pose_ = true;
    }
    node_handle_.param(node_name_+"/do_rectify", do_rectify_, false);

    // Map loading parameters
    node_handle_.param(node_name_+"/load_map", load_map_, false);
    node_handle_.param(node_name_+"/map_file_name", map_file_name_, std::string("map_file_name"));
    
    if(publish_map_) {
        map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(node_name_+"/map", 1);
    }
    if(publish_rendered_image_) {
        rendered_image_publisher_ = image_transport_.advertise(node_name_+"/rendered_image", 1);
    }
    if(publish_pose_) {
        pose_publisher_visualization_ = node_handle_.advertise<geometry_msgs::PoseStamped>(node_name_+"/pose", 1);
        pose_publisher_mavros_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/ifs/mavros/vision_pose/pose", 1);
    }

    // load SLAM initialization parameters
    node_handle_.param(node_name_+"/vocabulary_file", vocabulary_file_name_, std::string("you can't run if you don't have one"));
    node_handle_.param(node_name_+"/settings_file", settings_file_name_, std::string("you can't run if you don't have one"));
}

void node::initialize_orb_slam3() {
    // initialize ORB-SLAM

    // Turn localization mode ON by default, one should manually switch to mapping mode using dynamic reconfigure to build map
    orb_slam_ = new ORB_SLAM3::System(vocabulary_file_name_, settings_file_name_, sensor_type_);
    // orb_slam_->TurnLocalizationMode(true);
    last_tracking_state_ = orb_slam_->GetTrackingState();

    // load the initial vehicle pose if the map is given
    if (load_map_ && !load_initial_pose(map_file_name_)) {
        ROS_ERROR("[ORB_SLAM3_ROS] Initial vehicle pose is not loaded. Terminating node.");
        ros::shutdown();
        return;
    }
}

void node::initialize_ros_post_slam() {
    // service server for saving map
    save_map_service_ = node_handle_.advertiseService(node_name_+"/save_map", &node::service_save_map, this);

    // service server for setting slam mode
    set_localization_mode_service_ = node_handle_.advertiseService(node_name_+"/set_localization_mode", &node::service_set_localization_mode, this);

    // service server for rescaling
    rescale_service_ = node_handle_.advertiseService(node_name_+"/rescale", &node::service_rescale_map, this);

    // service server for minimum observations per point
    set_mopp_service_ = node_handle_.advertiseService(node_name_+"/set_mopp", &node::service_set_minimum_observations_per_point, this);
}

bool node::load_initial_pose(const std::string &file_name) {
    std::ifstream in(file_name + "_initial_tf.bin");
    if (!in.is_open()) {
        ROS_ERROR("[ORB_SLAM3_ROS] Initial tf file %s not found.", (file_name + "_initial_tf.bin").c_str());
        return false;
    }
    try {
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> tf_map_to_vehicle_init_;
        ia >> tf_vehicle_init_to_map_;
        in.close();
        print_transform_info(tf_map_to_vehicle_init_, "Loaded initial vehicle pose");
        return true;
    } catch (...) {
        return false;
    }
}

void node::check_slam_initialized(const int tracking_state) {
    if (!slam_initialized_) {
        if (tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK && last_tracking_state_ != ORB_SLAM3::Tracking::eTrackingState::OK) {
            if (!load_map_){
                // Get the initial vehicle pose, if the map is not given
                geometry_msgs::TransformStamped tf_voxl_to_map = tfBuffer_.lookupTransform("voxl", "map", ros::Time(0));
                tf2::fromMsg(tf_voxl_to_map.transform, tf_vehicle_init_to_map_);
                tf_map_to_vehicle_init_ = tf_vehicle_init_to_map_.inverse();

                print_transform_info(tf_map_to_vehicle_init_, "Initial vehicle pose");
            }
            slam_initialized_ = true;
            ROS_INFO("[ORB_SLAM3_ROS] SLAM initialized.");
        }
        last_tracking_state_ = tracking_state;
    }
}

void node::update_latest_linux_monotonic_clock_time() {
    clock_gettime(CLOCK_MONOTONIC, &ts);
    latest_image_time_linux_monotonic_.sec = ts.tv_sec;
    latest_image_time_linux_monotonic_.nsec = ts.tv_nsec;
}

void node::publish_pose_and_image() {
    if(publish_rendered_image_) {
        publish_rendered_image(orb_slam_->GetRenderedImage());
    }
    if (!slam_initialized_) {
        return;
    }
    if(publish_pose_) {
        publish_pose();
    }
}

void node::publish_periodicals() {
    if (!slam_initialized_) {
        return;
    }
    if (publish_map_) {
        publish_point_cloud(orb_slam_->GetAllMapPoints());
    }
}

bool node::service_save_map(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res){
    std::string file_name = ros::package::getPath("orb_SLAM3_ros") + "/resource/" + req.name;
    //TODO: use ATLAS
    // res.success = orb_slam_->SaveMap(file_name + ".bin");
    res.success = false;
    if (!res.success) {
        ROS_ERROR("[ORB_SLAM3_ROS] Map could not be saved.");
    }
    if (!save_initial_pose(file_name)){
        ROS_ERROR("[ORB_SLAM3_ROS] Initial tf could not be saved.");
        res.success = false;
    }
    ROS_INFO("[ORB_SLAM3_ROS] Map & Initial pose saved.");
    return res.success;
}

bool node::service_set_localization_mode(orb_slam3_ros::SetLocalizationMode::Request &req, orb_slam3_ros::SetLocalizationMode::Response &res){
    try {
        orb_slam_->TurnLocalizationMode(req.localization_mode);
    } catch (...) {
        res.success = false;
        return false;
    }
    res.success = (orb_slam_->GetLocalizationMode() == req.localization_mode);
    ROS_INFO("[ORB_SLAM3_ROS] %s mode %s", req.localization_mode ? "Localization" : "Mapping", res.success ? "set" : "not set");
    return res.success;
}

bool node::service_rescale_map(orb_slam3_ros::RescaleMap::Request &req, orb_slam3_ros::RescaleMap::Response &res){
    if (req.scale_factor < 1e-1F) {
        return false;
    }
    if (req.scale_factor > 1e2F) {
        return false;
    }
    scale_factor_ = req.scale_factor;
    ROS_INFO("[ORB_SLAM3_ROS] Map rescaled by %.3f", scale_factor_);
    return true;
}

bool node::service_set_minimum_observations_per_point(orb_slam3_ros::SetMopp::Request &req, orb_slam3_ros::SetMopp::Response &res){
    if (req.min_observations_per_point < 1) {
        return false;
    }
    if (req.min_observations_per_point > 10) {
        return false;
    }
    min_observations_per_point_ = req.min_observations_per_point;
    ROS_INFO("[ORB_SLAM3_ROS] Minimum observations per point set to %d", req.min_observations_per_point);
    return true;
}

bool node::save_initial_pose(const std::string &file_name) {
    std::ofstream out(file_name + "_initial_tf.bin", std::ios_base::binary);
    if (!out.is_open()) {
        ROS_ERROR("[ORB_SLAM3_ROS] Initial tf file %s not found.", (file_name + "_initial_tf.bin").c_str());
        return false;
    }
    try {
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << tf_map_to_vehicle_init_;
        oa << tf_vehicle_init_to_map_;
        out.close();
        print_transform_info(tf_map_to_vehicle_init_, "Saved initial vehicle pose");
        return true;
    } catch (...) {
        return false;
    }
}

void node::publish_rendered_image(cv::Mat image) {
    std_msgs::Header header;
    header.stamp = latest_image_time_internal_use_;
    header.frame_id = "tracking_camera";
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(image_msg);
}

void node::publish_pose() {
    update_local_tf();
    const tf2::Transform latest_global_tf_enu = tf_map_to_vehicle_init_ * latest_local_tf_;
    tf2::Stamped<tf2::Transform> latest_stamped_tf_visualization(latest_global_tf_enu, latest_image_time_internal_use_, "map");
    tf2::Stamped<tf2::Transform> latest_stamped_tf_mavros(latest_global_tf_enu, latest_image_time_linux_monotonic_, "map");

    geometry_msgs::PoseStamped pose_msg;
    pose_publisher_visualization_.publish(tf2::toMsg(latest_stamped_tf_visualization, pose_msg));
    pose_publisher_mavros_.publish(tf2::toMsg(latest_stamped_tf_mavros, pose_msg));

    if (publish_tf_) {
        geometry_msgs::TransformStamped latest_tf_stamped = tf2::toMsg(latest_stamped_tf_visualization);
        latest_tf_stamped.child_frame_id = namespace_.empty() ? DEFAULT_NAMESPACE : namespace_; // TODO: body to camera frame

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(latest_tf_stamped);
    }
}

void node::update_local_tf() {
    latest_local_tf_ = convert_orb_homogeneous_to_local_enu(latest_Tcw_);
    // print_transform_info(latest_local_tf_, "latest_local_tf");
}

void node::publish_point_cloud(std::vector<ORB_SLAM3::MapPoint*> map_points) {
    if (map_points.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 point_cloud;

    const int num_channels = 3;
    point_cloud.header.stamp = latest_image_time_internal_use_;
    point_cloud.header.frame_id = "map";;
    point_cloud.height = 1;
    point_cloud.width = map_points.size();
    point_cloud.is_bigendian = false;
    point_cloud.is_dense = true;
    point_cloud.point_step = num_channels * sizeof(float);
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        point_cloud.fields[i].name = channel_id[i];
        point_cloud.fields[i].offset = i * sizeof(float);
        point_cloud.fields[i].count = 1;
        point_cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    point_cloud.data.resize(point_cloud.row_step * point_cloud.height);

    float data_array[num_channels];
    unsigned char *cloud_data_ptr = &(point_cloud.data[0]);

#ifndef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
    tf2::Vector3 point;
    tf2::Vector3 point_transformed;
    tf2::Transform tf_map_to_vehicle_init_flu(tf_map_to_vehicle_init_.getBasis() * R_rdf_to_flu_, tf_map_to_vehicle_init_.getOrigin());
#endif

    for (unsigned int i = 0; i < map_points.size(); i++) {
        if (map_points[i] == nullptr) {
            continue;
        }
        if (map_points[i]->nObs >= min_observations_per_point_) {
            const Eigen::Vector3f point_world = map_points[i]->GetWorldPos();
            if (point_world.isZero() || point_world.hasNaN()) {
                continue;
            }
#ifdef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
            // RDF -> ENU (ORB-SLAM3 -> ROS), see convert_orb_homogeneous_to_local_enu
            data_array[0] = point_world.x();
            data_array[1] = -point_world.y();
            data_array[2] = -point_world.z();
#else
            point.setValue(point_world.x(), point_world.y(), point_world.z());
            point_transformed = tf_map_to_vehicle_init_flu * (point * scale_factor_);
            data_array[0] = point_transformed.getX();
            data_array[1] = point_transformed.getY();
            data_array[2] = point_transformed.getZ();
#endif
            memcpy(cloud_data_ptr+(i*point_cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    map_publisher_.publish(point_cloud);
}

tf2::Transform node::convert_orb_homogeneous_to_local_enu(cv::Mat Tcw){
    /*
        Conversion from ORB_SLAM3 to ROS(ENU)
            ORB-SLAM3:  X-right,    Y-down,     Z-forward
            ROS ENU:    X-east,     Y-north,    Z-up

        but one is local frame (camera) and the other is inertial (world or map).. this is wierd
        Do we have to assume that the initial pose (of ORB_SLAM3) is identity? and aligned to ENU? -> yes, and thus FLU=ENU

        Transformation: 
            X_enu = Z_orb
            Y_enu = -X_orb
            Z_enu = -Y_orb
    */

    if (Tcw.empty()) {
        return latest_local_tf_;
    }

    // Convert rotation matrix to tf2::Matrix3x3
    tf2::Matrix3x3 tf2_Rcw(
        Tcw.at<float>(0, 0), Tcw.at<float>(0, 1), Tcw.at<float>(0, 2),
        Tcw.at<float>(1, 0), Tcw.at<float>(1, 1), Tcw.at<float>(1, 2),
        Tcw.at<float>(2, 0), Tcw.at<float>(2, 1), Tcw.at<float>(2, 2)
    );
    tf2::Vector3 tf2_tcw(Tcw.at<float>(0, 3), Tcw.at<float>(1, 3), Tcw.at<float>(2, 3));
    tf2_tcw *= scale_factor_;
    tf2::Matrix3x3 tf2_Rwc = tf2_Rcw.transpose();
    tf2::Vector3 tf2_twc = -(tf2_Rwc * tf2_tcw);

    // Rotation matrix should be left multiplied by R_rdf_to_flu_ and right multiplied by its transpose to get correct rotation
    return tf2::Transform(R_rdf_to_flu_ * tf2_Rwc * R_flu_to_rdf_, R_rdf_to_flu_ * tf2_twc);
}

void node::print_transform_info(const tf2::Transform &tf, const std::string &name) {
    tf2::Vector3 t = tf.getOrigin();

    double roll, pitch, yaw;
    tf.getBasis().getRPY(roll, pitch, yaw);

    ROS_INFO("[ORB_SLAM3_ROS] %s: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", name.c_str(),
        t.getX(), t.getY(), t.getZ(), roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
}

}  // namespace orb_slam3_ros