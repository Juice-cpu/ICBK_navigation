#ifndef PATROL__PATROL_NODE_HPP_
#define PATROL__PATROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"
#include <chrono>
#include <memory>
#include <functional>

namespace patrol
{
class PatrolNode : public rclcpp::Node
{
public:
    PatrolNode(const rclcpp::NodeOptions & options);
    ~PatrolNode() = default;

private:
    // Correct order for initialization in constructor
    // Serial communication should be initialized after the boolean variable
    bool moving_to_end_point_;  // True if moving to the end point
    serial::Serial serial_port_;  // Initialize serial communication last

    // Define other member variables
    geometry_msgs::msg::Twist start_point_;
    geometry_msgs::msg::Twist end_point_;

    // Timer for periodic patrol updates
    rclcpp::TimerBase::SharedPtr patrol_timer_;

    // Publishers for cmd_vel and patrol status messages
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr patrol_publisher_;

    // Callback for patrol logic
    void patrol_callback();

    // Publish velocity commands
    void publishCmdVel(const geometry_msgs::msg::Twist &msg);

    // Send patrol data to external controller
    void sendPatrolDataToController(const std_msgs::msg::String &msg);
};

}  // namespace patrol

#endif  // PATROL__PATROL_NODE_HPP_
