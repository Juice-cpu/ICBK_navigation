#ifndef REAL_VEL_TRANSFORM_EC_COMM_NODE_HPP_
#define REAL_VEL_TRANSFORM_EC_COMM_NODE_HPP_

#include <serial/serial.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace real_vel_transform
{

const std::string CMD_VEL_CHASSIS_TOPIC = "/cmd_vel_chassis";

struct __attribute__((packed)) NavToEcFrame
{
  uint8_t head[2] = {'S', 'P'};
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  uint8_t home_arrived_flag = 0;
  uint16_t crc16 = 0;
};
static_assert(sizeof(NavToEcFrame) <= 64, "NavToEcFrame should stay small for serial transport");

struct __attribute__((packed)) EcToNavFrame
{
  uint8_t head[2] = {'S', 'P'};
  float x = 0.0f;
  float y = 0.0f;
  float angle = 0.0f;
  uint8_t game_type = 0;
  uint8_t game_progress = 0;
  uint16_t time_remaining = 0;
  uint16_t sentry_hp = 0;
  uint16_t ally_outpost_hp = 0;
  uint8_t ec_leave_flag = 0;
  uint8_t ec_home_flag = 0;
  uint16_t crc16 = 0;
};
static_assert(sizeof(EcToNavFrame) <= 64, "EcToNavFrame should stay small for serial transport");

class EcCommNode : public rclcpp::Node
{
public:
  explicit EcCommNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void configureSerialPort();
  void closeSerialPort();
  void handleSerialFailure(const std::string & context, const std::string & error_text);
  void tryReconnect();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pollSerial();
  void processRxBuffer();
  void publishEcFrame(const EcToNavFrame & frame);
  void watchdogCallback();
  void sendVelocityFrame(const geometry_msgs::msg::Twist & msg);
  uint16_t calculateCrc16(const uint8_t * data, size_t length) const;
  bool checkCrc16(const uint8_t * data, size_t length) const;

  serial::Serial serial_port_;
  bool serial_available_;
  std::string serial_port_name_;
  int baud_rate_;
  rclcpp::Time last_reconnect_attempt_time_;
  double reconnect_interval_sec_;
  double cmd_vel_timeout_sec_;

  uint8_t home_arrived_flag_;
  bool home_arrived_state_;
  bool require_go_home_decision_for_arrived_;
  bool has_cmd_vel_;
  bool watchdog_active_;
  rclcpp::Time last_cmd_vel_time_;
  std::vector<uint8_t> rx_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_type_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_time_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr hp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr ally_outpost_hp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ec_leave_flag_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ec_home_flag_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr home_arrived_pub_;
};

}  // namespace real_vel_transform

#endif  // REAL_VEL_TRANSFORM_EC_COMM_NODE_HPP_
