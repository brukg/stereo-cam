#ifndef STEREO_CAM_DEPTH_NODE_HPP_
#define STEREO_CAM_DEPTH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stereo.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace stereo_cam {

class DepthNode : public rclcpp::Node
{
public:
    explicit DepthNode(const rclcpp::NodeOptions& options, const std::string& name = "depth_node");

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
    void depthToPointCloud(const cv::Mat& depth_map, 
                          const std_msgs::msg::Header& header);
    void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void initializeStereoMatcher();

    // Base class pointer for stereo matcher
    cv::Ptr<cv::StereoMatcher> stereo_matcher_;
    bool use_sgbm_ = false;  // Flag to choose between BM and SGBM

    // Stereo matcher parameters
    struct StereoParams {
        int num_disparities = 128;
        int block_size = 21;
        int pre_filter_size = 63;
        int pre_filter_cap = 11;
        int min_disparity = 0;
        int texture_threshold = 16;
        int uniqueness_ratio = 5;
        int speckle_window_size = 41;
        int speckle_range = 32;
        int disp12_max_diff = 1;
        // SGBM specific parameters
        int p1 = 0;
        int p2 = 0;
        bool use_mode_hh = false;
    } stereo_params_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Synchronizer> sync_;

    // Camera parameters
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double baseline_ = 0.0;
    bool camera_info_received_ = false;
};

}  // namespace stereo_cam

#endif  // STEREO_CAM_DEPTH_NODE_HPP_ 