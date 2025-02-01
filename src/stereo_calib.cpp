#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include <mutex>

namespace stereo_cam {

// Forward declarations
void StereoCalibration(std::vector<std::string>imagelist, int numCornersVer, int numCornersHor, int numSquares, int ShowChessCorners);
static bool readStringList(const std::string& filename, std::vector<std::string>& l);
void ShowMatchResult(cv::Mat& srcImg, std::vector<cv::KeyPoint>& srcKeypoint, cv::Mat& dstImg,
	std::vector<cv::KeyPoint>& dstKeypoint, std::vector<cv::DMatch>& goodMatch);

class StereoCalibrator : public rclcpp::Node {
public:
	StereoCalibrator() : Node("stereo_calibrator") {
		// Declare and get parameters with default values
		this->declare_parameter("num_corners_vertical", 6);
		this->declare_parameter("num_corners_horizontal", 4);
		this->declare_parameter("square_size_mm", 30);
		this->declare_parameter("show_chess_corners", true);

		num_corners_vertical_ = this->get_parameter("num_corners_vertical").as_int();
		num_corners_horizontal_ = this->get_parameter("num_corners_horizontal").as_int();
		square_size_mm_ = this->get_parameter("square_size_mm").as_int();
		show_chess_corners_ = this->get_parameter("show_chess_corners").as_bool();

		RCLCPP_INFO(get_logger(), "Calibration parameters:");
		RCLCPP_INFO(get_logger(), "Vertical corners: %d", num_corners_vertical_);
		RCLCPP_INFO(get_logger(), "Horizontal corners: %d", num_corners_horizontal_);
		RCLCPP_INFO(get_logger(), "Square size: %d mm", square_size_mm_);

		// Create directories if they don't exist
		image_dir_ = "left_right_image";
		std::filesystem::create_directories(image_dir_);

		// Set up synchronized subscribers
		left_sub_.subscribe(this, "camera/left/image_raw");
		right_sub_.subscribe(this, "camera/right/image_raw");

		sync_ = std::make_shared<Synchronizer>(
			SyncPolicy(10),
			left_sub_,
			right_sub_
		);

		sync_->registerCallback(
			std::bind(&StereoCalibrator::sync_callback, this,
				std::placeholders::_1,
				std::placeholders::_2)
		);

		// Create display timer (30 Hz)
		display_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(33),
			std::bind(&StereoCalibrator::display_callback, this));

		RCLCPP_INFO(get_logger(), "Stereo calibrator initialized");
		RCLCPP_INFO(get_logger(), "Press 'q' to exit");
		RCLCPP_INFO(get_logger(), "Press 'k' to save current stereo pair");
		RCLCPP_INFO(get_logger(), "Press 'c' to start calibration");
	}

	~StereoCalibrator() {
		cv::destroyAllWindows();
	}

private:
	cv::Mat display_image_;
	std::mutex display_mutex_;
	rclcpp::TimerBase::SharedPtr display_timer_;
	bool running_{true};

	void display_callback() {
		cv::Mat current_image;
		{
			std::lock_guard<std::mutex> lock(display_mutex_);
			if (!display_image_.empty()) {
				current_image = display_image_.clone();
			}
		}
		
		if (!current_image.empty()) {
			cv::imshow("Stereo Cameras", current_image);
			char key = cv::waitKey(1);

			if (key == 'q') {
				RCLCPP_INFO(get_logger(), "Shutting down...");
				rclcpp::shutdown();
			}
			else if (key == 'k') {
				save_images(left_image_, right_image_);
			}
			else if (key == 'c') {
				RCLCPP_INFO(get_logger(), "Starting calibration...");
				perform_calibration();
			}
		}
	}

	void sync_callback(
		const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
		const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
	{
		try {
			left_image_ = cv_bridge::toCvShare(left_msg, "bgr8")->image;
			right_image_ = cv_bridge::toCvShare(right_msg, "bgr8")->image;

			cv::Mat stereo_image;
			cv::hconcat(left_image_, right_image_, stereo_image);

			// Resize for display if width is larger than 640
			cv::Mat display_image = stereo_image.clone();
			if (display_image.cols > 640) {
				double scale = 640.0 / display_image.cols;
				cv::resize(display_image, display_image, cv::Size(), scale, scale);
			}

			// Update display image thread-safely
			{
				std::lock_guard<std::mutex> lock(display_mutex_);
				display_image_ = display_image.clone();
			}
		}
		catch (const std::exception& e) {
			RCLCPP_ERROR(get_logger(), "Error processing images: %s", e.what());
		}
	}

	void save_images(const cv::Mat& left_image, const cv::Mat& right_image) {
		std::string left_name = image_dir_ + "/left" + std::to_string(image_count_) + ".jpg";
		std::string right_name = image_dir_ + "/right" + std::to_string(image_count_) + ".jpg";

		cv::imwrite(left_name, left_image);
		cv::imwrite(right_name, right_image);
		RCLCPP_INFO(get_logger(), "Saved image pair %d", image_count_);
		image_count_++;

		// Generate/update XML file for later use
		std::ofstream xml_file("stereo_calib.xml");
		xml_file << "<?xml version=\"1.0\"?>\n<opencv_storage>\n<imagelist>\n";
		for (int i = 0; i < image_count_; i++) {
			xml_file << "./" << image_dir_ << "/left" << i << ".jpg\n";
			xml_file << "./" << image_dir_ << "/right" << i << ".jpg\n";
		}
		xml_file << "</imagelist>\n</opencv_storage>";
		xml_file.close();
		
		RCLCPP_INFO(get_logger(), "Updated stereo_calib.xml");
	}

	void perform_calibration() {
		if (image_count_ > 0) {
			// If we just captured images, use them directly
			std::vector<std::string> image_list;
			for (int i = 0; i < image_count_; i++) {
				image_list.push_back(image_dir_ + "/left" + std::to_string(i) + ".jpg");
				image_list.push_back(image_dir_ + "/right" + std::to_string(i) + ".jpg");
			}
			StereoCalibration(image_list, 
				num_corners_vertical_,
				num_corners_horizontal_,
				square_size_mm_,
				show_chess_corners_);
		} else {
			// Try to load existing XML file
			std::vector<std::string> image_list;
			if (!readStringList("stereo_calib.xml", image_list)) {
				RCLCPP_ERROR(get_logger(), "No images captured and no stereo_calib.xml found!");
				return;
			}
			StereoCalibration(image_list,
				num_corners_vertical_,
				num_corners_horizontal_,
				square_size_mm_,
				show_chess_corners_);
		}
	}

	message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
	message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;

	using SyncPolicy = message_filters::sync_policies::ApproximateTime<
		sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
	using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
	std::shared_ptr<Synchronizer> sync_;

	std::string image_dir_;
	int image_count_{0};

	// Add these member variables
	int num_corners_vertical_;
	int num_corners_horizontal_;
	int square_size_mm_;
	bool show_chess_corners_;

	// Add these to store the latest images for saving
	cv::Mat left_image_;
	cv::Mat right_image_;
};

// Move all the calibration-related functions and structs here
struct CalibrationParam
{
	cv::Mat intrinsic_left;
	cv::Mat distCoeffs_left;
	cv::Mat	intrinsic_right;
	cv::Mat distCoeffs_right;
	cv::Mat R;
	cv::Mat T;
	cv::Mat R_L;
	cv::Mat R_R;
	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
	cv::Rect validROIL, validROIR;

	CalibrationParam() {
		intrinsic_left = cv::Mat();
		distCoeffs_left = cv::Mat();
		intrinsic_right = cv::Mat();
		distCoeffs_right = cv::Mat();
		R = cv::Mat();
		T = cv::Mat();
		R_L = cv::Mat();
		R_R = cv::Mat();
		P1 = cv::Mat();
		P2 = cv::Mat();
		Q = cv::Mat();
	}
};

void WriteObjectYml(const char* filename, const char* variablename, const cv::Mat &source)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << variablename << source;
	fs.release();
}

void ReadObjectYml(const char* filename, CalibrationParam&Calibrationparam)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::runtime_error("Failed to open calibration file: " + std::string(filename));
	}
	fs["validROIL"] >> Calibrationparam.validROIL;
	fs["validROIR"] >> Calibrationparam.validROIR;
	fs["intrinsic_left"] >> Calibrationparam.intrinsic_left;
	fs["distCoeffs_left"] >> Calibrationparam.distCoeffs_left;
	fs["intrinsic_right"] >> Calibrationparam.intrinsic_right;
	fs["distCoeffs_right"] >> Calibrationparam.distCoeffs_right;
	fs["R"] >> Calibrationparam.R;
	fs["T"] >> Calibrationparam.T;
	fs["R_L"] >> Calibrationparam.R_L;
	fs["R_R"] >> Calibrationparam.R_R;
	fs["P1"] >> Calibrationparam.P1;
	fs["P2"] >> Calibrationparam.P2;
	fs["Q"] >> Calibrationparam.Q;
	fs.release();
}

static bool readStringList(const std::string& filename, std::vector<std::string>& l)
{
	l.resize(0);
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
		return false;
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((std::string)*it);
	return true;
}

void StereoCalibration(std::vector<std::string>imagelist, int numCornersVer, int numCornersHor, int numSquares, int ShowChessCorners)
{
	if (numCornersVer <= 0 || numCornersHor <= 0 || numSquares <= 0) {
		throw std::invalid_argument("Invalid chessboard parameters");
	}

	if (imagelist.size() % 2 != 0)
	{
		std::cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	std::vector<std::vector<cv::Point2f>> image_leftPoints, image_rightPoints;
	std::vector<std::vector<cv::Point3f>> objectPoints;
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
	cv::Mat gray_l, gray_r;
	cv::Mat image_l, image_r;
	std::vector<cv::Point3f> obj;
	for (int i = 0; i < numCornersHor; i++)
	{
		for (int j = 0; j < numCornersVer; j++)
		{
			obj.push_back(cv::Point3f((float)j * numSquares, (float)i * numSquares, 0));
		}	
	}
	cv::Size s1, s2;
	for (size_t i = 0; i < imagelist.size()/2; i++)
	{
		
		image_l = cv::imread(imagelist[2*i]);
		image_r = cv::imread(imagelist[2*i+1]);
		if (image_l.empty() || image_r.empty()) {
			std::cerr << "Failed to load images: " << imagelist[2*i] << " or " << imagelist[2*i+1] << std::endl;
			continue;
		}
		
		s1 = image_l.size();
		s2 = image_r.size();

		cvtColor(image_l, gray_l, cv::COLOR_BGR2GRAY);
		cvtColor(image_r, gray_r, cv::COLOR_BGR2GRAY);
		std::vector<cv::Point2f> corners_r;
		std::vector<cv::Point2f> corners_l;
		bool ret1 = findChessboardCorners(gray_r, cv::Size(numCornersVer, numCornersHor), corners_r, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		bool ret2 = findChessboardCorners(gray_l, cv::Size(numCornersVer, numCornersHor), corners_l, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		if (ret1&&ret2&&ShowChessCorners)
		{
			cornerSubPix(gray_l, corners_l, cv::Size(5, 5), cv::Size(-1, -1), criteria);
			drawChessboardCorners(image_l, cv::Size(numCornersVer, numCornersHor), corners_l, ret1);
			imshow("ChessboardCorners", image_l);
			std::cout << imagelist[2 * i] << std::endl;
			cv::waitKey(0);
			cornerSubPix(gray_r, corners_r, cv::Size(5, 5), cv::Size(-1, -1), criteria);
			drawChessboardCorners(image_r, cv::Size(numCornersVer, numCornersHor), corners_r, ret2);
			imshow("ChessboardCorners", image_r);
			std::cout << imagelist[2 * i+1] << std::endl;
			cv::waitKey(0);
		}
		if (ret1&&ret2)
		{
			image_rightPoints.push_back(corners_r);
			image_leftPoints.push_back(corners_l);
			objectPoints.push_back(obj);
		}

	}
	if (objectPoints.empty() || image_leftPoints.empty() || image_rightPoints.empty()) {
		throw std::runtime_error("No valid chessboard patterns found in images");
	}

	cv::Mat intrinsic_left = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs_left;
	std::vector<cv::Mat> rvecs_l;
	std::vector<cv::Mat> tvecs_l;

	intrinsic_left.ptr<float>(0)[0] = 1;
	intrinsic_left.ptr<float>(1)[1] = 1;
	cv::calibrateCamera(objectPoints, image_leftPoints, s1, intrinsic_left, distCoeffs_left, rvecs_l, tvecs_l);
	cv::Mat intrinsic_right = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs_right;
	std::vector<cv::Mat> rvecs_r;
	std::vector<cv::Mat> tvecs_r;
	cv::Mat R_total;
	cv::Vec3d T_total;
	intrinsic_right.ptr<float>(0)[0] = 1;
	intrinsic_right.ptr<float>(1)[1] = 1;
	cv::calibrateCamera(objectPoints, image_rightPoints, s2, intrinsic_right, distCoeffs_right, rvecs_r, tvecs_r);
	cv::Mat R_L;
	cv::Mat R_R;
	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
	cv::Rect validROIL, validROIR;
	cv::Mat E;
	cv::Mat F;
	cv::Mat R;
	cv::Mat T;
	std::cout << "Stereo Calibration start!" <<std::endl;
	double rms = cv::stereoCalibrate(objectPoints, image_leftPoints, image_rightPoints,intrinsic_left, distCoeffs_left,intrinsic_right, distCoeffs_right,
		cv::Size(1920,1080), R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));                                                                                                                        
	std::cout << "Stereo Calibration done with RMS error = " << rms << std::endl;

	std::cout << "Starting Rectification" << std::endl;
	stereoRectify(intrinsic_left, distCoeffs_left, intrinsic_right, distCoeffs_right, s1, R, T, R_L, R_R, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, s1, &validROIL, &validROIR);
	std::cout << "Rectification Done" << std::endl;

	// Save calibration
	std::string package_path = ament_index_cpp::get_package_share_directory("stereo_cam");
	std::string calib_file = package_path + "/config/StereoCalibration.yaml";
	std::cout << "Save Calibration to " << calib_file << std::endl;
	cv::FileStorage fs(calib_file, cv::FileStorage::WRITE);
	fs << "image_width" << s1.width;
	fs << "image_height" << s1.height;
	fs << "intrinsic_left" << intrinsic_left;
	fs << "distCoeffs_left" << distCoeffs_left;
	fs << "intrinsic_right" << intrinsic_right;
	fs << "distCoeffs_right" << distCoeffs_right;
	fs << "R" << R;
	fs << "T" << T;
	fs << "R_L" << R_L;
	fs << "R_R" << R_R;
	fs << "P1" << P1;
	fs << "P2" << P2;
	fs << "Q" << Q;
	fs << "validROIL" << validROIL;
	fs << "validROIR" << validROIR;
	fs.release();
	std::cout << "Done Calibration" << std::endl;

	// Compute rectification maps
	cv::Mat map1x, map1y, map2x, map2y;
	cv::initUndistortRectifyMap(intrinsic_left, distCoeffs_left, R_L, P1, s1, CV_32FC1, map1x, map1y);
	cv::initUndistortRectifyMap(intrinsic_right, distCoeffs_right, R_R, P2, s1, CV_32FC1, map2x, map2y);

	// Now load a pair and show rectified matches
	cv::Mat img1 = cv::imread(imagelist[0]);
	cv::Mat img2 = cv::imread(imagelist[1]);
	
	// Rectify the images
	cv::Mat rect1, rect2;
	cv::remap(img1, rect1, map1x, map1y, cv::INTER_LINEAR);
	cv::remap(img2, rect2, map2x, map2y, cv::INTER_LINEAR);

	// Detect features
	cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;
	
	detector->detectAndCompute(rect1, cv::Mat(), keypoints1, descriptors1);
	detector->detectAndCompute(rect2, cv::Mat(), keypoints2, descriptors2);

	// Match features
	std::vector<cv::DMatch> good_matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
	std::vector<std::vector<cv::DMatch>> knn_matches;
	matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

	// Filter good matches
	for (size_t i = 0; i < knn_matches.size(); i++) {
		if (knn_matches[i][0].distance < 0.7f * knn_matches[i][1].distance) {
			good_matches.push_back(knn_matches[i][0]);
		}
	}

	// Show matches
	ShowMatchResult(rect1, keypoints1, rect2, keypoints2, good_matches);

	return;
}

void ShowMatchResult(cv::Mat&srcImg, std::vector<cv::KeyPoint>&srcKeypoint, cv::Mat&dstImg,
	std::vector<cv::KeyPoint>&dstKeypoint, std::vector<cv::DMatch>&goodMatch)
{
	cv::drawKeypoints(srcImg, srcKeypoint, srcImg);
	cv::drawKeypoints(dstImg, dstKeypoint, dstImg);

	cv::Mat img_matches;
	cv::drawMatches(srcImg, srcKeypoint, dstImg, dstKeypoint, goodMatch, img_matches,
		cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	// Resize for display if width is larger than 640
	cv::Mat display_img = img_matches.clone();
	if (display_img.cols > 640) {
		double scale = 640.0 / display_img.cols;
		cv::resize(display_img, display_img, cv::Size(), scale, scale);
	}

	imshow("Good Matches", display_img);
	cv::imwrite("img_matches.jpg", img_matches);  // Save original size
	std::cout << "Good MatchPoint Num is :" << goodMatch.size() << std::endl;
	cv::waitKey(0);
	return;
}


} // namespace stereo_cam

int main(int argc,char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<stereo_cam::StereoCalibrator>();
	
	try {
		rclcpp::spin(node);
	}
	catch (const std::exception& e) {
		RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
	}

	cv::destroyAllWindows();
	rclcpp::shutdown();
	return 0;
}
