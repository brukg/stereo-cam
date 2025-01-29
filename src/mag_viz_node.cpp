#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MagVizNode : public rclcpp::Node {
public:
    MagVizNode() : Node("mag_viz_node") {
        // Subscribe to magnetometer data
        mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
            "imu/mag", 10,
            std::bind(&MagVizNode::mag_callback, this, std::placeholders::_1));

        // Publisher for visualization marker
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("mag_vector", 10);
    }

private:
    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "mag_vector";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Start point of the arrow
        marker.points.resize(2);
        marker.points[0].x = 0.0;
        marker.points[0].y = 0.0;
        marker.points[0].z = 0.0;

        // Scale the magnetic field values for better visualization
        float scale = 1.0;  // Adjust this value to make the arrow bigger/smaller
        marker.points[1].x = msg->magnetic_field.x * scale;
        marker.points[1].y = msg->magnetic_field.y * scale;
        marker.points[1].z = msg->magnetic_field.z * scale;

        // Arrow properties
        marker.scale.x = 0.02;  // shaft diameter
        marker.scale.y = 0.04;  // head diameter
        marker.scale.z = 0.1;   // head length

        // Color (blue)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MagVizNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 