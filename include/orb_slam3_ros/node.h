#ifndef ORB_SLAM3_ROS_NODE_H_
#define ORB_SLAM3_ROS_NODE_H_

#include <ros/ros.h>

// packages
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// tf
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// services
#include <orb_slam3_ros/SaveMap.h>
#include <orb_slam3_ros/SetLocalizationMode.h>
#include <orb_slam3_ros/RescaleMap.h>
#include <orb_slam3_ros/SetMopp.h>

// ORB_SLAM3
#include "System.h"

#define DEFAULT_NAMESPACE "ids"

namespace orb_slam3_ros {

// Frame convention: ORB-SLAM3=RDF, ROS=ENU, assuming initialliy facing east
const tf2::Matrix3x3 R_rdf_to_flu_  (0, 0, 1,
                                    -1, 0, 0,
                                    0,-1, 0);
const tf2::Matrix3x3 R_flu_to_rdf_  (R_rdf_to_flu_.transpose());
const tf2::Matrix3x3 R_flu_to_frd_  (1, 0, 0,
                                    0, -1, 0,
                                    0, 0, -1);
const tf2::Matrix3x3 R_frd_to_flu_  (R_flu_to_frd_.transpose());
const tf2::Matrix3x3 R_ned_to_enu_  (0, 1, 0,
                                    1, 0, 0,
                                    0, 0,-1);
const tf2::Matrix3x3 R_enu_to_ned_  (R_ned_to_enu_.transpose());
class node {
    public:
        node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM3::System::eSensor sensor_type);
        ~node();

    protected:
        // slam-derived
        cv::Mat latest_Tcw_;
        ros::Time latest_image_time_internal_use_;
        ros::Time latest_image_time_linux_monotonic_; // time to be sent to MAVROS
        struct timespec ts; // for clock_gettime in linux, which will be relayed to MAVROS
        ORB_SLAM3::System *orb_slam_;
        bool slam_initialized_;

        void initialize_node();
        void check_slam_initialized(const int tracking_state);
        void update_latest_linux_monotonic_clock_time();
        void publish_pose_and_image();
        void publish_periodicals();

    private:
        void initialize_ros_pre_slam();
        void initialize_orb_slam3();
        void initialize_ros_post_slam();

        void publish_rendered_image(cv::Mat image);
        void publish_pose();
        void publish_point_cloud(std::vector<ORB_SLAM3::MapPoint*> map_points);

        bool service_save_map(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res);
        bool service_set_localization_mode(orb_slam3_ros::SetLocalizationMode::Request &req, orb_slam3_ros::SetLocalizationMode::Response &res);
        bool service_rescale_map(orb_slam3_ros::RescaleMap::Request &req, orb_slam3_ros::RescaleMap::Response &res);
        bool service_set_minimum_observations_per_point(orb_slam3_ros::SetMopp::Request &req, orb_slam3_ros::SetMopp::Response &res);

        tf2::Transform convert_orb_homogeneous_to_local_enu(cv::Mat Tcw); // to the initial frame of the ORB-SLAM3
        void update_local_tf();
        void print_transform_info(const tf2::Transform &tf, const std::string &name);
        bool load_initial_pose(const std::string &file_name);
        bool save_initial_pose(const std::string &file_name);

        // node
        std::string node_name_;
        std::string namespace_;
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;

        // publisher & subscriber
        ros::Publisher map_publisher_;
        ros::Publisher pose_publisher_visualization_;
        ros::Publisher pose_publisher_mavros_;
        image_transport::Publisher rendered_image_publisher_;

        // tf
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        // services & related variables
        ros::ServiceServer save_map_service_;
        ros::ServiceServer set_localization_mode_service_;
        ros::ServiceServer rescale_service_;
        ros::ServiceServer set_mopp_service_; // minimum observations per point
        float scale_factor_;
        int min_observations_per_point_;

        // ros flags
        bool publish_map_;
        bool publish_pose_;
        bool publish_tf_;
        bool publish_rendered_image_;
        bool do_rectify_;

        // vehicle pose when ORB-slam3 is initialized
        tf2::Transform tf_map_to_vehicle_init_;
        tf2::Transform tf_vehicle_init_to_map_;

        // slam parameters
        std::string vocabulary_file_name_;
        std::string settings_file_name_;
        ORB_SLAM3::System::eSensor sensor_type_;
        std::string map_file_name_;
        bool load_map_;

        // slam-derived
        int last_tracking_state_;
        tf2::Transform latest_local_tf_; // taking the initial frame as the reference frame
        tf2::Stamped<tf2::Transform> latest_stamped_local_tf_;
};

}  // namespace orb_slam3_ros

#endif // ORB_SLAM3_ROS_NODE_H_