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
    auto depth_node = std::make_shared<stereo_cam::DepthNode>(options, "depth_node");
    
    executor.add_node(stereo_node);
    executor.add_node(depth_node);
    
    stereo_node->on_configure();
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
} 