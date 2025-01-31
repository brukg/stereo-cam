#include "stereo_cam/stereo_node.hpp"

namespace stereo_cam {

// Definition of static member
const std::vector<CameraConfig::Resolution> CameraConfig::SUPPORTED_MODES = {
    {3280, 2464, 21.19}, // Full resolution
    {1920, 1080, 47.57}, // 1080p
    {1280, 720, 41.85},  // 720p
    {640, 480, 206.65}    // VGA
};

StereoNode::StereoNode(const rclcpp::NodeOptions& options, const std::string& name)
    : Node(name, options) {
    
    // Only declare and get parameters in constructor
    this->declare_parameter("width", rclcpp::ParameterValue(640));
    this->declare_parameter("height", rclcpp::ParameterValue(480));
    this->declare_parameter("frame_rate", rclcpp::ParameterValue(30));
    this->declare_parameter("frame_id", "camera_frame");
    this->declare_parameter("resolution_preset", "1080p");
    this->declare_parameter("imu_rate", 100);
    this->declare_parameter("left_camera_info_url", "");
    this->declare_parameter("right_camera_info_url", "");

    // Get resolution from preset
    std::string preset = this->get_parameter("resolution_preset").as_string();
    RCLCPP_INFO(get_logger(), "Resolution preset: %s", preset.c_str());

    // Default to VGA if no match
    width_ = 1920;
    height_ = 1080;
    frame_rate_ = 30;

    for (const auto& mode : CameraConfig::SUPPORTED_MODES) {
        if (preset == "1080p" && mode.width == 1920) {
            width_ = mode.width;
            height_ = mode.height;
            frame_rate_ = mode.fps;
            RCLCPP_INFO(get_logger(), "Setting 1080p mode: %dx%d @ %d fps", width_, height_, frame_rate_);
            break;
        } else if (preset == "720p" && mode.width == 1280) {
            width_ = mode.width;
            height_ = mode.height;
            frame_rate_ = mode.fps;
            RCLCPP_INFO(get_logger(), "Setting 720p mode: %dx%d @ %d fps", width_, height_, frame_rate_);
            break;
        } else if (preset == "full" && mode.width == 3280) {
            width_ = mode.width;
            height_ = mode.height;
            frame_rate_ = mode.fps;
            RCLCPP_INFO(get_logger(), "Setting full resolution mode: %dx%d @ %d fps", width_, height_, frame_rate_);
            break;
        }
    }
    RCLCPP_INFO(get_logger(), "Final width: %d, height: %d, frame rate: %d", width_, height_, frame_rate_);

    // // Only override if explicitly set in the launch file
    // if (this->get_parameter("width").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    //     width_ = this->get_parameter("width").as_int();
    //     RCLCPP_INFO(get_logger(), "Explicitly set width: %d", width_);
    // }
    // if (this->get_parameter("height").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    //     height_ = this->get_parameter("height").as_int();
    //     RCLCPP_INFO(get_logger(), "Explicitly set height: %d", height_);
    // }
    // if (this->get_parameter("frame_rate").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    //     frame_rate_ = this->get_parameter("frame_rate").as_int();
    // }

    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    imu_rate_ = this->get_parameter("imu_rate").as_int();
    
    // Get camera info URLs
    left_camera_info_url_ = this->get_parameter("left_camera_info_url").as_string();
    right_camera_info_url_ = this->get_parameter("right_camera_info_url").as_string();

    // Register callback for when node is fully up
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&StereoNode::on_configure, this));
}

void StereoNode::on_configure() {
    // Stop the timer
    timer_.reset();
    
    // Now safe to use shared_from_this()
    initialize();
}

void StereoNode::initialize() {
    // Create publishers
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    left_pub_ = it_->advertise("camera/left/image_raw", 10);
    right_pub_ = it_->advertise("camera/right/image_raw", 10);

    // Initialize camera info publishers
    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info", 10);

    // Initialize cameras
    left_cam_ = std::make_unique<lccv::PiCamera>(0);
    right_cam_ = std::make_unique<lccv::PiCamera>(1);

    configure_cameras();

    // Start cameras
    left_cam_->startVideo();
    right_cam_->startVideo();

    // Create timer for image capture
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frame_rate_),
        std::bind(&StereoNode::timer_callback, this));

    // Initialize IMU
    imu_ = std::make_unique<ICM20948>();
    if (!imu_->init()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU");
        return;
    }

    // Configure IMU
    imu_->configureAccel(AccelRange::G4);
    imu_->configureGyro(GyroRange::DPS500);
    imu_->enableMagnetometer(true);

    // Create IMU publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    // Create IMU timer (100Hz default)
    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / imu_rate_),
        std::bind(&StereoNode::imu_callback, this));

    // Initialize camera info managers with their respective URLs
    left_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, 
        "left_camera",
        left_camera_info_url_
    );

    right_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this,
        "right_camera", 
        right_camera_info_url_
    );

    if (!left_info_manager_->loadCameraInfo(left_camera_info_url_)) {
        RCLCPP_WARN(get_logger(), "Failed to load left camera calibration");
    }
    if (!right_info_manager_->loadCameraInfo(right_camera_info_url_)) {
        RCLCPP_WARN(get_logger(), "Failed to load right camera calibration");
    }

    RCLCPP_INFO(this->get_logger(), "Stereo camera node initialized");
}

StereoNode::~StereoNode() {
    if (left_cam_) left_cam_->stopVideo();
    if (right_cam_) right_cam_->stopVideo();
}

void StereoNode::configure_cameras() {
    RCLCPP_INFO(get_logger(), "Configuring cameras with resolution: %dx%d @ %d fps", 
                width_, height_, frame_rate_);

    // Configure left camera
    left_cam_->options->video_width = width_;
    left_cam_->options->video_height = height_;
    left_cam_->options->framerate = 30;

    // Configure right camera
    right_cam_->options->video_width = width_;
    right_cam_->options->video_height = height_;
    right_cam_->options->framerate = 30;

    // Print current configuration
    RCLCPP_INFO(get_logger(), "Left camera options: %dx%d @ %d fps", 
                left_cam_->options->video_width,
                left_cam_->options->video_height,
                left_cam_->options->framerate);

    RCLCPP_INFO(get_logger(), "Right camera options: %dx%d @ %d fps", 
                right_cam_->options->video_width,
                right_cam_->options->video_height,
                right_cam_->options->framerate);

    // Initialize camera info messages
    left_info_.header.frame_id = frame_id_;
    left_info_.width = width_;
    left_info_.height = height_;

    right_info_ = left_info_;  // Copy basic parameters
}

void StereoNode::timer_callback() {
    cv::Mat left_frame, right_frame;
    
    // Capture images
    if (!left_cam_->getVideoFrame(left_frame, 1000) || 
        !right_cam_->getVideoFrame(right_frame, 1000)) {
        RCLCPP_WARN(this->get_logger(), "Failed to capture stereo images");
        return;
    }

    publish_images(left_frame, right_frame);
}

void StereoNode::publish_images(const cv::Mat& left_img, const cv::Mat& right_img) {
    auto stamp = this->now();
    
    // Convert and publish left image
    sensor_msgs::msg::Image::SharedPtr left_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_img).toImageMsg();
    left_msg->header.stamp = stamp;
    left_msg->header.frame_id = frame_id_;
    left_pub_.publish(left_msg);

    // Get and publish left camera info
    sensor_msgs::msg::CameraInfo left_info = left_info_manager_->getCameraInfo();
    left_info.header.stamp = stamp;
    left_info.header.frame_id = frame_id_;
    left_info_pub_->publish(left_info);

    // Convert and publish right image
    sensor_msgs::msg::Image::SharedPtr right_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();
    right_msg->header.stamp = stamp;
    right_msg->header.frame_id = frame_id_;
    right_pub_.publish(right_msg);

    // Get and publish right camera info
    sensor_msgs::msg::CameraInfo right_info = right_info_manager_->getCameraInfo();
    right_info.header.stamp = stamp;
    right_info.header.frame_id = frame_id_;
    right_info_pub_->publish(right_info);
}

void StereoNode::imu_callback() {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    
    auto stamp = this->now();

    // Read IMU data
    if (imu_->readAccel(ax, ay, az) && imu_->readGyro(gx, gy, gz)) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "imu_link";

        // Accelerometer data already in g's, convert to m/s^2
        imu_msg.linear_acceleration.x = ax * 9.81;
        imu_msg.linear_acceleration.y = ay * 9.81;
        imu_msg.linear_acceleration.z = az * 9.81;

        // Gyro data already in deg/s, convert to rad/s
        imu_msg.angular_velocity.x = gx * M_PI / 180.0;
        imu_msg.angular_velocity.y = gy * M_PI / 180.0;
        imu_msg.angular_velocity.z = gz * M_PI / 180.0;

        // Set orientation quaternion
        imu_msg.orientation.w = q0;
        imu_msg.orientation.x = q1;
        imu_msg.orientation.y = q2;
        imu_msg.orientation.z = q3;

        imu_pub_->publish(imu_msg);
    }

    // Read magnetometer data
    if (imu_->readMag(mx, my, mz)) {
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = stamp;
        mag_msg.header.frame_id = "imu_link";

        mag_msg.magnetic_field.x = mx;
        mag_msg.magnetic_field.y = my;
        mag_msg.magnetic_field.z = mz;

        mag_pub_->publish(mag_msg);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to read magnetometer data");
    }
}

} // namespace stereo_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_cam::StereoNode) 