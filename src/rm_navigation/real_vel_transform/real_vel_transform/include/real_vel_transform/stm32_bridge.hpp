#ifndef REAL_VEL_TRANSFORM_STM32_BRIDGE_HPP_
#define REAL_VEL_TRANSFORM_STM32_BRIDGE_HPP_

#include <serial/serial.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <memory>
#include <string>

namespace real_vel_transform
{

const std::string STM32_TX_FRAME_TOPIC = "/stm32_bridge/tx_frame";
const std::string STM32_RX_FRAME_TOPIC = "/stm32_bridge/rx_frame";

class Stm32Bridge : public rclcpp::Node
{
public:
  explicit Stm32Bridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void configureSerialPort(const std::string & port, int baud_rate);
  void handleSerialFailure(const std::string & context, const std::string & error_text);
  void tryReconnect();
  void txFrameCallback(const std_msgs::msg::String::SharedPtr msg);
  void pollSerial();

  serial::Serial serial_port_;
  bool serial_available_;
  std::string serial_port_name_;
  int baud_rate_;
  rclcpp::Time last_reconnect_attempt_time_;
  double reconnect_interval_sec_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_frame_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_frame_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace real_vel_transform

#endif  // REAL_VEL_TRANSFORM_STM32_BRIDGE_HPP_
