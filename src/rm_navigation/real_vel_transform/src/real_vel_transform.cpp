#include "real_vel_transform/real_vel_transform.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
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
: Node("real_vel_transform", options),
  serial_port_("/dev/stm32_robot_base", 115200, serial::Timeout::simpleTimeout(1000))
{
  RCLCPP_INFO(get_logger(), "Start RealVelTransform!");
  latest_hp_ = 0;
  posture_mode_ = 1;
  has_target_msg_ = false;
  has_cmd_vel_ = false;
  serial_watchdog_active_ = false;
  serial_watchdog_timeout_sec_ = 0.3;
  last_cmd_vel_time_ = this->get_clock()->now();

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

  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1, std::bind(&RealVelTransform::localPoseCallback, this, std::placeholders::_1));

  hp_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
    SENTRY_HP_TOPIC, 10, std::bind(&RealVelTransform::hpCallback, this, std::placeholders::_1));

  target_sub_ = this->create_subscription<std_msgs::msg::String>(
    TARGET_TOPIC, rclcpp::SensorDataQoS(),
    std::bind(&RealVelTransform::targetCallback, this, std::placeholders::_1));

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
    RCLCPP_WARN(get_logger(), "Received empty or invalid PoseArray message");
    return;
  }

  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  double teb_angle = tf2::getYaw(selected_pose.orientation);
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

    double angle_diff = -current_angle_;

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
      "Serial watchdog timeout: cmd_vel stale for %.3f s (> %.3f s), forcing stop frame",
      age, serial_watchdog_timeout_sec_);
  }
  serial_watchdog_active_ = true;
}

void RealVelTransform::hpCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  latest_hp_ = msg->data;
}

void RealVelTransform::targetCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const char * raw = msg->data.c_str();
  char * end = nullptr;
  const double target_flag = std::strtod(raw, &end);
  has_target_msg_ = (end != raw) && (target_flag > 0.5);

  if (has_cmd_vel_) {
    sendVelocityToController(last_controller_cmd_vel_);
  }
}

void RealVelTransform::sendVelocityToController(const geometry_msgs::msg::Twist & msg)
{
  try {
    if (latest_hp_ < kHpEscapeThreshold) {
      posture_mode_ = 2;
    } else if (latest_hp_ < kHpThreshold) {
      posture_mode_ = 3;
    } else if (has_target_msg_) {
      posture_mode_ = 1;
    } else {
      posture_mode_ = 3;
    }

    std::stringstream ss;
    ss << "$"
       << std::fixed << std::setprecision(5)
       << std::setw(8) << msg.linear.x
       << std::setw(8) << msg.linear.y
       << std::setw(8) << msg.angular.z
       << posture_mode_
       << "V";
    std::string data = ss.str();
    RCLCPP_INFO(get_logger(), "Sending to STM32: %ld", data.size());
    RCLCPP_INFO(get_logger(), "Sending to STM32: %s", data.c_str());
    RCLCPP_INFO(get_logger(), "Latest HP: %d", latest_hp_);
    RCLCPP_INFO(get_logger(), "Posture Mode: %d", posture_mode_);

    serial_port_.write(data);
  } catch (const serial::IOException & e) {
    RCLCPP_ERROR(get_logger(), "Serial communication error: %s", e.what());
  }
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_vel_transform::RealVelTransform>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(real_vel_transform::RealVelTransform)
