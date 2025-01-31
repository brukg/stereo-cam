#include "stereo_cam/depth_node.hpp"

namespace stereo_cam {

DepthNode::DepthNode(const rclcpp::NodeOptions& options, const std::string& name)
: Node(name, options)
{
    // Initialize stereo matcher
    stereo_matcher_ = cv::StereoSGBM::create(
        0,      // minDisparity
        128,    // numDisparities
        11,     // blockSize
        8 * 3 * 11 * 11,     // P1
        32 * 3 * 11 * 11,    // P2
        1,      // disp12MaxDiff
        10,     // uniquenessRatio
        100,    // speckleWindowSize
        32      // speckleRange
    );

    // Create publisher for depth map with intra-process communication
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/depth", rclcpp::QoS(10).reliable());

    // Create subscribers for stereo images with intra-process communication
    left_sub_.subscribe(this, "camera/left/image_raw", rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, "camera/right/image_raw", rmw_qos_profile_sensor_data);

    // Set up synchronizer for stereo pairs
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&DepthNode::imageCallback, this,
        std::placeholders::_1, std::placeholders::_2));
}

void DepthNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
{
    try {
        cv_bridge::CvImageConstPtr left_cv = cv_bridge::toCvShare(left_msg);
        cv_bridge::CvImageConstPtr right_cv = cv_bridge::toCvShare(right_msg);

        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_cv->image, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_cv->image, right_gray, cv::COLOR_BGR2GRAY);

        cv::Mat disparity;
        stereo_matcher_->compute(left_gray, right_gray, disparity);

        // Convert to float and scale
        cv::Mat depth_map;
        disparity.convertTo(depth_map, CV_32F, 1.0/16.0);

        // Create and publish depth message
        auto depth_msg = cv_bridge::CvImage(left_msg->header,
                                          sensor_msgs::image_encodings::TYPE_32FC1,
                                          depth_map).toImageMsg();
        depth_pub_->publish(*depth_msg);
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

}  // namespace stereo_cam 