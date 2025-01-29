#include "stereo_cam/stereo_node.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

namespace stereo_cam {

// Definition of static member
const std::vector<CameraConfig::Resolution> CameraConfig::SUPPORTED_MODES = {
    {3280, 2464, 15}, // Full resolution
    {1920, 1080, 30}, // 1080p
    {1280, 720, 60},  // 720p
    {640, 480, 90}    // VGA
};

StereoNode::StereoNode(const rclcpp::NodeOptions& options)
    : Node("stereo_cam_node", options) {
    
    // Declare parameters with correct types
    this->declare_parameter("width", 640);
    this->declare_parameter("height", 480);
    this->declare_parameter("frame_rate", 30);  // Explicitly integer
    this->declare_parameter("frame_id", "camera_frame");
    this->declare_parameter("resolution_preset", "1080p");
    this->declare_parameter("imu_rate", 100);  // Explicitly integer

    // Get parameters
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    frame_rate_ = this->get_parameter("frame_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    imu_rate_ = this->get_parameter("imu_rate").as_int();
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

    RCLCPP_INFO(this->get_logger(), "Stereo camera node initialized");
}

StereoNode::~StereoNode() {
    if (left_cam_) left_cam_->stopVideo();
    if (right_cam_) right_cam_->stopVideo();
}

void StereoNode::configure_cameras() {
    // Configure left camera
    left_cam_->options->video_width = width_;
    left_cam_->options->video_height = height_;
    left_cam_->options->framerate = frame_rate_;

    // Configure right camera
    right_cam_->options->video_width = width_;
    right_cam_->options->video_height = height_;
    right_cam_->options->framerate = frame_rate_;

    // Initialize camera info messages
    left_info_.header.frame_id = frame_id_;
    left_info_.width = width_;
    left_info_.height = height_;
    // Add camera matrix, distortion, etc.

    right_info_ = left_info_;  // Copy basic parameters
    // Modify right camera specific parameters
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

    // Convert and publish right image
    sensor_msgs::msg::Image::SharedPtr right_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();
    right_msg->header.stamp = stamp;
    right_msg->header.frame_id = frame_id_;
    right_pub_.publish(right_msg);
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

        // Update quaternion using AHRS algorithm
        float halfT = 1.0f / (2.0f * imu_rate_);
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;
        static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
        const float Kp = 4.50f;
        const float Ki = 1.0f;

        // ... (AHRS algorithm implementation)

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