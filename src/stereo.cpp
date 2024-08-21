#include "orb_slam3_ros/stereo.h"
#include "orb_slam3_core/include/Converter.h"

namespace orb_slam3_ros {

stereo::stereo(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM3::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // Initiazte tracking callback of Stereo ORB-SLAM3
    left_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/left/image_raw", 1);
    right_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/right/image_raw", 1);

    synchronizer_ = new message_filters::Synchronizer<PolicyTimeSync>(PolicyTimeSync(10), *left_image_subscriber_, *right_image_subscriber_);
    synchronizer_->registerCallback(boost::bind(&stereo::callback_image, this, _1, _2));

    // Set up a timer to publish point cloud
    timer_ = node_handle.createTimer(ros::Duration(2.0), &stereo::callback_timer, this);
}

stereo::~stereo() {
    ROS_INFO("[Stereo] Terminating Stereo.");
    delete left_image_subscriber_;
    delete right_image_subscriber_;
    delete synchronizer_;
}

void stereo::callback_image(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr_left, cv_ptr_right;
    try {
        cv_ptr_left = cv_bridge::toCvShare(msg_left);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try {
        cv_ptr_right = cv_bridge::toCvShare(msg_right);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // mark the time of the last processed image
    latest_image_time_internal_use_ = msg_left->header.stamp;
    update_latest_linux_monotonic_clock_time();

    // pass images to ORB-SLAM
    latest_Tcw_ = ORB_SLAM3::Converter::toCvMat(
        ORB_SLAM3::Converter::toSE3Quat(
            orb_slam_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image, latest_image_time_internal_use_.toSec())
        )
        );

    check_slam_initialized(orb_slam_->GetTrackingState());
    publish_pose_and_image();
}

void stereo::callback_timer(const ros::TimerEvent&) {
    publish_periodicals();
}

}  // namespace orb_slam3_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam3_stereo");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam3_ros::stereo slam_node(node_handle, image_transport, ORB_SLAM3::System::STEREO);

    ros::spin();
    return 0;
}