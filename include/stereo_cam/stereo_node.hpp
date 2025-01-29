#ifndef STEREO_CAM_STEREO_NODE_HPP
#define STEREO_CAM_STEREO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "lccv/lccv.hpp"
#include "stereo_cam/icm20948.hpp"

namespace stereo_cam {

struct CameraConfig {
    static constexpr int MAX_WIDTH = 3280;
    static constexpr int MAX_HEIGHT = 2464;
    static constexpr double FOCAL_LENGTH = 2.6; // mm
    static constexpr double BASELINE = 0.06; // meters (60mm)
    static constexpr double FOV_H = 73.0; // degrees
    static constexpr double FOV_V = 50.0; // degrees
    
    struct Resolution {
        int width;
        int height;
        int fps;
    };
    
    static const std::vector<Resolution> SUPPORTED_MODES;  // Declaration only
};

class StereoNode : public rclcpp::Node {
public:
    explicit StereoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~StereoNode();

    void initialize();

private:
    void timer_callback();
    void configure_cameras();
    void publish_images(const cv::Mat& left_img, const cv::Mat& right_img);

    // Camera instances
    std::unique_ptr<lccv::PiCamera> left_cam_;
    std::unique_ptr<lccv::PiCamera> right_cam_;

    // Publishers
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

    // Camera info messages
    sensor_msgs::msg::CameraInfo left_info_;
    sensor_msgs::msg::CameraInfo right_info_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    int width_;
    int height_;
    int frame_rate_;
    std::string frame_id_;

    // IMU members
    std::unique_ptr<ICM20948> imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    
    void imu_callback();
    bool configure_imu();
    
    // IMU parameters
    AccelRange accel_range_;
    GyroRange gyro_range_;
    int imu_rate_;

    // Camera info managers
    std::unique_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
    std::string left_camera_info_url_;
    std::string right_camera_info_url_;
};

} // namespace stereo_cam

#endif // STEREO_CAM_STEREO_NODE_HPP 