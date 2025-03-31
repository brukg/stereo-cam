#include <rclcpp/rclcpp.hpp>
#include "stereo_cam/stereo_node.hpp"
#include "stereo_cam/depth_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto stereo_node = std::make_shared<stereo_cam::StereoNode>(options, "stereo_node");
    
    // Add node to executor first
    executor.add_node(stereo_node);
    
    // Let the node initialize and get parameters
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // Only create depth node if enabled
    std::shared_ptr<stereo_cam::DepthNode> depth_node;
    bool enable_depth = stereo_node->get_parameter("enable_depth").as_bool();
    RCLCPP_INFO(stereo_node->get_logger(), "Enable depth: %d", enable_depth);
    if (enable_depth) {
        RCLCPP_INFO(stereo_node->get_logger(), "Creating depth node");
        depth_node = std::make_shared<stereo_cam::DepthNode>(options, "depth_node");
        executor.add_node(depth_node);
    }
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
} 