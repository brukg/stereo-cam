#ifndef STEREO_CAM_DEPTH_NODE_HPP_
#define STEREO_CAM_DEPTH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stereo.hpp>

namespace stereo_cam {

class DepthNode : public rclcpp::Node
{
public:
    explicit DepthNode(const rclcpp::NodeOptions& options, const std::string& name = "depth_node");

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);

    cv::Ptr<cv::StereoSGBM> stereo_matcher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Synchronizer> sync_;
};

}  // namespace stereo_cam

#endif  // STEREO_CAM_DEPTH_NODE_HPP_ 