#include <rclcpp/rclcpp.hpp>
#include "stereo_cam/stereo_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_cam::StereoNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 