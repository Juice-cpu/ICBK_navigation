#include "real_vel_transform/real_vel_transform.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace real_vel_transform
{
RealVelTransform::RealVelTransform(const rclcpp::NodeOptions & options)
: Node("real_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start RealVelTransform!");
  home_arrived_flag_ = 0;
  has_cmd_vel_ = false;
  serial_watchdog_active_ = false;
  serial_watchdog_timeout_sec_ = 0.3;
  last_cmd_vel_time_ = this->get_clock()->now();
  current_angle_ = 0.0;
  base_link_angle_ = 0.0;

  this->declare_parameter<float>("spin_speed", 0.0);
  this->get_parameter("spin_speed", spin_speed_);
  this->declare_parameter<double>("serial_watchdog_timeout_sec", 0.3);
  this->get_parameter("serial_watchdog_timeout_sec", serial_watchdog_timeout_sec_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&RealVelTransform::cmdVelCallback, this, std::placeholders::_1));

  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  stm32_tx_frame_pub_ = this->create_publisher<std_msgs::msg::String>(
    STM32_TX_FRAME_TOPIC, rclcpp::QoS(rclcpp::KeepLast(10)));

  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1, std::bind(&RealVelTransform::localPoseCallback, this, std::placeholders::_1));

  home_arrived_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    HOME_ARRIVED_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&RealVelTransform::homeArrivedCallback, this, std::placeholders::_1));

  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&RealVelTransform::publishTransform, this));

  serial_watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RealVelTransform::serialWatchdogCallback, this));
}

void RealVelTransform::localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid local plan");
    return;
  }

  const size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  const double teb_angle = tf2::getYaw(selected_pose.orientation);
  current_angle_ = teb_angle - base_link_angle_;
}

void RealVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  has_cmd_vel_ = true;
  serial_watchdog_active_ = false;
  last_cmd_vel_time_ = this->get_clock()->now();

  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

    const double angle_diff = -current_angle_;

    geometry_msgs::msg::Twist aft_tf_vel;
    aft_tf_vel.angular.z = (msg->angular.z != 0.0) ? spin_speed_ : 0.0;
    aft_tf_vel.linear.x = msg->linear.x * std::cos(angle_diff) + msg->linear.y * std::sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * std::sin(angle_diff) + msg->linear.y * std::cos(angle_diff);

    cmd_vel_chassis_pub_->publish(aft_tf_vel);

    last_controller_cmd_vel_ = aft_tf_vel;
    sendVelocityToController(aft_tf_vel);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get transform: %s", ex.what());
  }
}

void RealVelTransform::serialWatchdogCallback()
{
  if (!has_cmd_vel_) {
    return;
  }

  const double age = (this->get_clock()->now() - last_cmd_vel_time_).seconds();
  if (age <= serial_watchdog_timeout_sec_) {
    return;
  }

  geometry_msgs::msg::Twist stop_msg;
  cmd_vel_chassis_pub_->publish(stop_msg);
  sendVelocityToController(stop_msg);

  if (!serial_watchdog_active_) {
    RCLCPP_WARN(
      get_logger(),
      "STM32 watchdog timeout: cmd_vel stale for %.3f s (> %.3f s), forcing stop frame",
      age, serial_watchdog_timeout_sec_);
  }
  serial_watchdog_active_ = true;
}

void RealVelTransform::homeArrivedCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const uint8_t flag = msg->data > 0 ? 1 : 0;
  if (home_arrived_flag_ == flag) {
    return;
  }
  home_arrived_flag_ = flag;
  if (has_cmd_vel_) {
    sendVelocityToController(last_controller_cmd_vel_);
  }
}

void RealVelTransform::sendVelocityToController(const geometry_msgs::msg::Twist & msg)
{
  std::stringstream ss;
  ss << "￥"
     << std::fixed << std::setprecision(5)
     << std::setw(8) << msg.linear.x
     << std::setw(8) << msg.linear.y
     << std::setw(8) << msg.angular.z
     << static_cast<unsigned>(home_arrived_flag_)
     << "V";

  std_msgs::msg::String frame_msg;
  frame_msg.data = ss.str();
  stm32_tx_frame_pub_->publish(frame_msg);

  RCLCPP_INFO(get_logger(), "Publishing STM32 frame: %ld", frame_msg.data.size());
  RCLCPP_INFO(get_logger(), "Publishing STM32 frame: %s", frame_msg.data.c_str());
  RCLCPP_INFO(get_logger(), "Home Arrived Flag: %u", static_cast<unsigned>(home_arrived_flag_));
}

void RealVelTransform::publishTransform()
{
  const auto now = get_clock()->now();
  if (now.nanoseconds() == 0) {
    RCLCPP_WARN(get_logger(), "Clock not active yet, skip TF publish");
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_link_fake";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

}  // namespace real_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(real_vel_transform::RealVelTransform)
