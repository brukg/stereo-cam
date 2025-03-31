#include "stereo_cam/depth_node.hpp"
#include <std_msgs/msg/header.hpp>

namespace stereo_cam {

DepthNode::DepthNode(const rclcpp::NodeOptions& options, const std::string& name)
: Node(name, options)
{
    // Declare parameters
    this->declare_parameter("use_sgbm", false);
    this->declare_parameter("num_disparities", 128);
    this->declare_parameter("block_size", 21);
    this->declare_parameter("pre_filter_size", 9);
    this->declare_parameter("pre_filter_cap", 31);
    this->declare_parameter("min_disparity", 0);
    this->declare_parameter("texture_threshold", 10);
    this->declare_parameter("uniqueness_ratio", 15);
    this->declare_parameter("speckle_window_size", 100);
    this->declare_parameter("speckle_range", 32);
    this->declare_parameter("disp12_max_diff", 1);
    
    // Get parameters
    use_sgbm_ = this->get_parameter("use_sgbm").as_bool();
    stereo_params_.num_disparities = this->get_parameter("num_disparities").as_int();
    stereo_params_.block_size = this->get_parameter("block_size").as_int();
    stereo_params_.pre_filter_size = this->get_parameter("pre_filter_size").as_int();
    stereo_params_.pre_filter_cap = this->get_parameter("pre_filter_cap").as_int();
    stereo_params_.min_disparity = this->get_parameter("min_disparity").as_int();
    stereo_params_.texture_threshold = this->get_parameter("texture_threshold").as_int();
    stereo_params_.uniqueness_ratio = this->get_parameter("uniqueness_ratio").as_int();
    stereo_params_.speckle_window_size = this->get_parameter("speckle_window_size").as_int();
    stereo_params_.speckle_range = this->get_parameter("speckle_range").as_int();
    stereo_params_.disp12_max_diff = this->get_parameter("disp12_max_diff").as_int();

    if (use_sgbm_) {
        // SGBM specific parameters
        this->declare_parameter("p1", 8 * 3 * stereo_params_.block_size * stereo_params_.block_size);
        this->declare_parameter("p2", 32 * 3 * stereo_params_.block_size * stereo_params_.block_size);
        this->declare_parameter("use_mode_hh", false);
        
        stereo_params_.p1 = this->get_parameter("p1").as_int();
        stereo_params_.p2 = this->get_parameter("p2").as_int();
        stereo_params_.use_mode_hh = this->get_parameter("use_mode_hh").as_bool();
    }

    // Initialize stereo matcher with parameters
    initializeStereoMatcher();

    // Create publishers
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/depth/image_raw", rclcpp::QoS(10).reliable());
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "camera/depth/points", rclcpp::QoS(10).reliable());

    // Create subscribers
    left_sub_.subscribe(this, "camera/left/image_raw", rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, "camera/right/image_raw", rmw_qos_profile_sensor_data);

    // Create subscribers for camera info
    // left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    //     "camera/left/camera_info", 
    //     rclcpp::QoS(10).reliable(),
    //     std::bind(&DepthNode::leftCameraInfoCallback, this, std::placeholders::_1));
    
    right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/right/camera_info", 
        rclcpp::QoS(10).reliable(),
        std::bind(&DepthNode::rightCameraInfoCallback, this, std::placeholders::_1));

    // Set up synchronizer
    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&DepthNode::imageCallback, this,
        std::placeholders::_1, std::placeholders::_2));
}

void DepthNode::initializeStereoMatcher()
{
    if (use_sgbm_) {
        auto sgbm = cv::StereoSGBM::create(
            stereo_params_.min_disparity,
            stereo_params_.num_disparities,
            stereo_params_.block_size,
            stereo_params_.p1,
            stereo_params_.p2,
            stereo_params_.disp12_max_diff,
            stereo_params_.pre_filter_cap,
            stereo_params_.uniqueness_ratio,
            stereo_params_.speckle_window_size,
            stereo_params_.speckle_range,
            stereo_params_.use_mode_hh ? cv::StereoSGBM::MODE_HH : cv::StereoSGBM::MODE_SGBM
        );
        stereo_matcher_ = sgbm;
        RCLCPP_INFO(get_logger(), "Initialized SGBM stereo matcher");
    } else {
        auto bm = cv::StereoBM::create(
            stereo_params_.num_disparities,
            stereo_params_.block_size
        );
        bm->setPreFilterSize(stereo_params_.pre_filter_size);
        bm->setPreFilterCap(stereo_params_.pre_filter_cap);
        bm->setMinDisparity(stereo_params_.min_disparity);
        bm->setTextureThreshold(stereo_params_.texture_threshold);
        bm->setUniquenessRatio(stereo_params_.uniqueness_ratio);
        bm->setSpeckleWindowSize(stereo_params_.speckle_window_size);
        bm->setSpeckleRange(stereo_params_.speckle_range);
        bm->setDisp12MaxDiff(stereo_params_.disp12_max_diff);
        stereo_matcher_ = bm;
        RCLCPP_INFO(get_logger(), "Initialized BM stereo matcher");
    }
}

void DepthNode::rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (msg->p[3] != 0.0) {  // P[3] is -fx * baseline
        fx_ = msg->p[0];     // Use fx from right projection matrix
        fy_ = msg->p[5];     // Use fy from right projection matrix
        cx_ = msg->p[2];     // Use cx from right projection matrix
        cy_ = msg->p[6];     // Use cy from right projection matrix
        baseline_ = -msg->p[3] / fx_;  // Calculate baseline
        camera_info_received_ = true;
        
        RCLCPP_INFO(get_logger(), "Received right camera info: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, baseline=%.2f", 
                    fx_, fy_, cx_, cy_, baseline_);
        right_info_sub_.reset();
        left_info_sub_.reset();  // We don't need left info anymore
    }
}

void DepthNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
{
    if (!camera_info_received_) {
        RCLCPP_WARN(get_logger(), "Waiting for camera info...");
        return;
    }

    try {
        cv_bridge::CvImageConstPtr left_cv = cv_bridge::toCvShare(left_msg);
        cv_bridge::CvImageConstPtr right_cv = cv_bridge::toCvShare(right_msg);

        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_cv->image, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_cv->image, right_gray, cv::COLOR_BGR2GRAY);

        cv::Mat disparity;
        stereo_matcher_->compute(left_gray, right_gray, disparity);

        // Add debug output for disparity
        double min_disp, max_disp;
        cv::minMaxLoc(disparity, &min_disp, &max_disp);

        // Convert disparity to depth
        cv::Mat depth_map(disparity.size(), CV_32F);
        int valid_points = 0;
        
        for(int y = 0; y < disparity.rows; y++) {
            for(int x = 0; x < disparity.cols; x++) {
                float disp = static_cast<float>(disparity.at<short>(y, x)) / 16.0f;  // SGBM returns fixed-point disparity with 4 fractional bits
                if(disp > 0.1f) {  // Minimum disparity threshold
                    float depth = (fx_ * baseline_) / disp;  // in millimeters
                    if(depth > 0 && depth < 10000.0f) {  // Reasonable depth range (0-10m)
                        depth_map.at<float>(y, x) = depth;
                        valid_points++;
                    } else {
                        depth_map.at<float>(y, x) = 0.0f;
                    }
                } else {
                    depth_map.at<float>(y, x) = 0.0f;
                }
            }
        }
        
       

        // Optionally visualize disparity (helps with debugging)
        cv::Mat disp_vis;
        disparity.convertTo(disp_vis, CV_8U, 255.0/2048.0);

        // Create and publish depth message
        auto depth_msg = cv_bridge::CvImage(left_msg->header,
                                          sensor_msgs::image_encodings::TYPE_32FC1,
                                          depth_map).toImageMsg();
        depth_pub_->publish(*depth_msg);

        // Generate and publish point cloud
        depthToPointCloud(depth_map, left_msg->header);
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

void DepthNode::depthToPointCloud(const cv::Mat& depth_map, 
                                 const std_msgs::msg::Header& header)
{
    // Create point cloud message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = header;
    cloud_msg.height = depth_map.rows;
    cloud_msg.width = depth_map.cols;
    
    // Set up fields
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[2].name = "z";
    
    for (size_t i = 0; i < 3; ++i) {
        cloud_msg.fields[i].offset = i * sizeof(float);
        cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[i].count = 1;
    }
    
    cloud_msg.is_bigendian = false;
    cloud_msg.point_step = 3 * sizeof(float);
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = false;
    
    // Allocate memory for point cloud data
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
    
    // Fill point cloud data
    float* data = reinterpret_cast<float*>(cloud_msg.data.data());
    for(int y = 0; y < depth_map.rows; y++) {
        for(int x = 0; x < depth_map.cols; x++) {
            int index = (y * depth_map.cols + x) * 3;
            float depth = depth_map.at<float>(y, x);
            
            if(depth > 0) {
                // Back-project 2D point to 3D
                data[index] = (x - cx_) * depth / fx_;     // X
                data[index + 1] = (y - cy_) * depth / fy_; // Y
                data[index + 2] = depth;                   // Z
            } else {
                data[index] = 0;
                data[index + 1] = 0;
                data[index + 2] = 0;
            }
        }
    }
    
    cloud_pub_->publish(cloud_msg);
}

}  // namespace stereo_cam 