#ifndef ORB_SLAM3_ROS_STEREO_H_
#define ORB_SLAM3_ROS_STEREO_H_

#include "orb_slam3_ros/node.h"

// Synchronization of a pair of images (e.g. left & right)
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace orb_slam3_ros {

class stereo: public node {
    public:
        stereo(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM3::System::eSensor sensor_type);
        ~stereo();
    private:
        void callback_image(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right);
        void callback_timer(const ros::TimerEvent&);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> PolicyTimeSync;
        message_filters::Synchronizer<PolicyTimeSync> *synchronizer_;
        message_filters::Subscriber<sensor_msgs::Image> *left_image_subscriber_;
        message_filters::Subscriber<sensor_msgs::Image> *right_image_subscriber_;
        ros::Timer timer_;
};

}  // namespace orb_slam3_ros

#endif // ORB_SLAM3_ROS_STEREO_H_