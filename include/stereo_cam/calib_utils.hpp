#ifndef STEREO_CAM_CALIB_UTILS_HPP_
#define STEREO_CAM_CALIB_UTILS_HPP_

#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace stereo_cam {

class CalibrationUtils {
public:
    static bool updateCameraInfo(
        const std::string& calib_file,
        sensor_msgs::msg::CameraInfo& left_info,
        sensor_msgs::msg::CameraInfo& right_info
    );

private:
    static void fillCameraInfo(
        const cv::Mat& K,
        const cv::Mat& D,
        const cv::Mat& R,
        const cv::Mat& P,
        sensor_msgs::msg::CameraInfo& info
    );
};

} // namespace stereo_cam

#endif 