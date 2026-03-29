#ifndef REAL_VEL_TRANSFORM_REAL_VEL_TRANSFORM_HPP_
#define REAL_VEL_TRANSFORM_REAL_VEL_TRANSFORM_HPP_

#include <serial/serial.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>

namespace real_vel_transform
{
const std::string CMD_VEL_TOPIC = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL = "/cmd_vel_chassis";
const std::string TRAJECTORY_TOPIC = "/local_plan";
const std::string SENTRY_HP_TOPIC = "/referee/sentry_hp";
const std::string TARGET_TOPIC = "/auto_aim_target_pos";
const int TF_PUBLISH_FREQUENCY = 20;

class RealVelTransform : public rclcpp::Node
{
public:
  explicit RealVelTransform(const rclcpp::NodeOptions & options);

private:
  serial::Serial serial_port_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr hp_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::string target_frame_;

  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr serial_watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::PoseStamped planner_local_pose_;

  double current_angle_;
  double base_link_angle_;
  float spin_speed_;

  uint16_t latest_hp_;
  int posture_mode_;
  bool has_target_msg_;
  geometry_msgs::msg::Twist last_controller_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  bool has_cmd_vel_;
  bool serial_watchdog_active_;
  double serial_watchdog_timeout_sec_;
  static constexpr uint16_t kHpThreshold = 300;
  static constexpr uint16_t kHpEscapeThreshold = 250;

  void localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void hpCallback(const std_msgs::msg::UInt16::SharedPtr msg);
  void targetCallback(const std_msgs::msg::String::SharedPtr msg);
  void sendVelocityToController(const geometry_msgs::msg::Twist & msg);
  void publishTransform();
  void serialWatchdogCallback();
};

}  // namespace real_vel_transform

#endif  // REAL_VEL_TRANSFORM_REAL_VEL_TRANSFORM_HPP_
