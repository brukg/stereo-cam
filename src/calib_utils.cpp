#include "stereo_cam/calib_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace stereo_cam {

bool CalibrationUtils::updateCameraInfo(
    const std::string& calib_file,
    sensor_msgs::msg::CameraInfo& left_info,
    sensor_msgs::msg::CameraInfo& right_info)
{
    // Handle package:// URLs
    std::string resolved_path = calib_file;
    if (calib_file.substr(0, 10) == "package://") {
        std::string package_name = calib_file.substr(10);
        size_t pos = package_name.find('/');
        if (pos == std::string::npos) {
            return false;
        }
        
        std::string package_path = package_name.substr(0, pos);
        std::string relative_path = package_name.substr(pos);
        
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory(package_path);
            resolved_path = package_share + relative_path;
        } catch (const std::exception& e) {
            return false;
        }
    }

    cv::FileStorage fs(resolved_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    // Read image size
    cv::Size image_size;
    fs["image_width"] >> image_size.width;
    fs["image_height"] >> image_size.height;

    // Read calibration matrices
    cv::Mat K1, D1, R1, P1;
    cv::Mat K2, D2, R2, P2;
    cv::Mat T;
    
    // Load left camera parameters
    fs["intrinsic_left"] >> K1;
    fs["distCoeffs_left"] >> D1;
    fs["R_L"] >> R1;
    fs["P1"] >> P1;
    
    // Load right camera parameters
    fs["intrinsic_right"] >> K2;
    fs["distCoeffs_right"] >> D2;
    fs["R_R"] >> R2;
    fs["P2"] >> P2;
    fs["T"] >> T;

    // Verify matrices were loaded correctly
    std::cout << "Loaded matrices from file:" << std::endl;
    std::cout << "intrinsic_left (K1): " << K1 << std::endl;
    std::cout << "intrinsic_right (K2): " << K2 << std::endl;
    std::cout << "P1: " << P1 << std::endl;
    std::cout << "P2: " << P2 << std::endl;

    // Add validation
    if (K1.empty() || K2.empty() || P1.empty() || P2.empty() || T.empty()) {
        std::cerr << "Failed to load calibration matrices" << std::endl;
        return false;
    }

    // Calculate and verify baseline
    double fx = P2.at<double>(0,0);     // focal length in pixels
    double tx = -P2.at<double>(0,3);    // baseline * focal length
    double baseline = tx / fx;           // baseline in mm
    
    if (baseline <= 0 || baseline > 1000) {  // sanity check
        std::cerr << "Invalid baseline value: " << baseline << " mm" << std::endl;
        return false;
    }

    std::cout << "Stereo Parameters:" << std::endl;
    std::cout << "Focal length (fx): " << fx << " pixels" << std::endl;
    std::cout << "Baseline: " << baseline << " mm" << std::endl;
    std::cout << "tx (baseline * fx): " << tx << std::endl;
    std::cout << "T vector from calibration: [" 
              << T.at<double>(0) << ", "
              << T.at<double>(1) << ", "
              << T.at<double>(2) << "] mm" << std::endl;

    // Scale calibration parameters if current resolution differs from calibration resolution
    if (left_info.width != image_size.width || left_info.height != image_size.height) {
        double sx = static_cast<double>(left_info.width) / image_size.width;
        double sy = static_cast<double>(left_info.height) / image_size.height;

        // Scale camera matrices
        K1.at<double>(0,0) *= sx;  // fx
        K1.at<double>(1,1) *= sy;  // fy
        K1.at<double>(0,2) *= sx;  // cx
        K1.at<double>(1,2) *= sy;  // cy

        K2.at<double>(0,0) *= sx;
        K2.at<double>(1,1) *= sy;
        K2.at<double>(0,2) *= sx;
        K2.at<double>(1,2) *= sy;

        // Scale projection matrices
        P1.at<double>(0,0) *= sx;
        P1.at<double>(1,1) *= sy;
        P1.at<double>(0,2) *= sx;
        P1.at<double>(1,2) *= sy;

        P2.at<double>(0,0) *= sx;
        P2.at<double>(1,1) *= sy;
        P2.at<double>(0,2) *= sx;
        P2.at<double>(1,2) *= sy;
    }
    std::cout << "image_size: " << image_size << std::endl;
    std::cout << "K1: " << K1 << std::endl;
    std::cout << "D1: " << D1 << std::endl;
    std::cout << "R1: " << R1 << std::endl;
    std::cout << "P1: " << P1 << std::endl;
    std::cout << "K2: " << K2 << std::endl;
    std::cout << "D2: " << D2 << std::endl;
    std::cout << "R2: " << R2 << std::endl;
    std::cout << "P2: " << P2 << std::endl;

    fillCameraInfo(K1, D1, R1, P1, left_info, image_size);
    fillCameraInfo(K2, D2, R2, P2, right_info, image_size);

    return true;
}

void CalibrationUtils::fillCameraInfo(
    const cv::Mat& K,
    const cv::Mat& D,
    const cv::Mat& R,
    const cv::Mat& P,
    sensor_msgs::msg::CameraInfo& info,
    const cv::Size& image_size)
{
    // Set image size
    info.height = image_size.height;
    info.width = image_size.width;
    
    // Set distortion model
    info.distortion_model = "plumb_bob";
    
    // Fill camera matrix (K)
    if (K.rows == 3 && K.cols == 3) {
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                info.k[i*3 + j] = K.at<double>(i,j);
            }
        }
    }

    // Fill distortion coefficients (D)
    info.d.clear();
    if (D.total() > 0) {
        info.d.resize(D.total());
        for(size_t i = 0; i < D.total(); i++) {
            info.d[i] = D.at<double>(i);
        }
    }

    // Fill rectification matrix (R)
    if (R.rows == 3 && R.cols == 3) {
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                info.r[i*3 + j] = R.at<double>(i,j);
            }
        }
    }

    // Fill projection matrix (P)
    if (P.rows == 3 && P.cols == 4) {
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 4; j++) {
                info.p[i*4 + j] = P.at<double>(i,j);
            }
        }
    }
}

} // namespace stereo_cam 