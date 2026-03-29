#include "patrol/patrol_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>  // Include the serial library
#include <chrono>
#include <sstream>
#include <iomanip>

namespace patrol
{
PatrolNode::PatrolNode(const rclcpp::NodeOptions & options)
: Node("patrol_node", options),
  moving_to_end_point_(true),  // Initialize boolean variable first
  serial_port_("/dev/myserial", 115200, serial::Timeout::simpleTimeout(1000))  // Initialize serial port second
{
    RCLCPP_INFO(get_logger(), "Start PatrolNode!");
    // Create a publisher for cmd_vel to send velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Create a publisher for patrol status
    patrol_publisher_ = this->create_publisher<std_msgs::msg::String>("patrol_topic", 10);

    // Initialize patrol points
    start_point_.linear.x = 0.0;
    start_point_.linear.y = 0.0;
    end_point_.linear.x = 4.1;  // Target position (5, 0)
    end_point_.linear.y = 5.5;
    // Set up a timer to call patrol_callback periodically
    patrol_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),  // Call every second
        std::bind(&PatrolNode::patrol_callback, this)
    );
}


void PatrolNode::patrol_callback()
{
    // Check if we need to reverse patrol direction
    if (moving_to_end_point_) {
        // Move towards the end point
        geometry_msgs::msg::Twist move_cmd;
        move_cmd.linear.x = 0.5;  // Move forward at 0.5 m/s
        publishCmdVel(move_cmd);

        // When near the end point, switch direction
        if (std::abs(end_point_.linear.x - start_point_.linear.x) < 0.1) {
            moving_to_end_point_ = false;
            RCLCPP_INFO(get_logger(), "Reached end point, now moving to start point!");
        }
    } else {
        // Move towards the start point
        geometry_msgs::msg::Twist move_cmd;
        move_cmd.linear.x = -0.5;  // Move backward at 0.5 m/s
        publishCmdVel(move_cmd);

        // When near the start point, switch direction
        if (std::abs(start_point_.linear.x - end_point_.linear.x) < 0.1) {
            moving_to_end_point_ = true;
            RCLCPP_INFO(get_logger(), "Reached start point, now moving to end point!");
        }
    }

    // Publish patrol status message
    auto message = std_msgs::msg::String();
    message.data = moving_to_end_point_ ? "Moving to end point" : "Moving to start point";
    patrol_publisher_->publish(message);

    // Send patrol status to external controller
    sendPatrolDataToController(message);
}

void PatrolNode::publishCmdVel(const geometry_msgs::msg::Twist &msg)
{
    // Publish velocity command
    cmd_vel_pub_->publish(msg);
}

void PatrolNode::sendPatrolDataToController(const std_msgs::msg::String &msg)
{
    try {
        if (!serial_port_.isOpen()) {
            // Ensure the serial port is open before sending data
            RCLCPP_ERROR(get_logger(), "Serial port is not open. Cannot send patrol data.");
            return;
        }

        // Prepare the data to send to the controller
        std::stringstream ss;
        ss << "Patrol Status: " << msg.data;
        std::string data = ss.str();
        
        // Send data over the serial port
        RCLCPP_INFO(get_logger(), "Sending to Controller: %s", data.c_str());
        serial_port_.write(data);  // Write data via serial communication
    } catch (const serial::IOException &e) {
        RCLCPP_ERROR(get_logger(), "Serial communication error: %s", e.what());
    }
}

}  // namespace patrol

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<patrol::PatrolNode>(rclcpp::NodeOptions()));  // Corrected line
  rclcpp::shutdown();
  return 0;
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the patrol node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(patrol::PatrolNode)
