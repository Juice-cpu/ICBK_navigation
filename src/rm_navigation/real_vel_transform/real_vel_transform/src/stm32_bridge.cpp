#include "real_vel_transform/stm32_bridge.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace real_vel_transform
{

Stm32Bridge::Stm32Bridge(const rclcpp::NodeOptions & options)
: Node("stm32_bridge", options),
  serial_available_(false),
  baud_rate_(115200),
  reconnect_interval_sec_(1.0)
{
  this->declare_parameter<std::string>("serial_port", "/dev/ACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<int>("poll_period_ms", 2);
  this->declare_parameter<double>("reconnect_interval_sec", 1.0);

  serial_port_name_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  int poll_period_ms = this->get_parameter("poll_period_ms").as_int();
  reconnect_interval_sec_ = this->get_parameter("reconnect_interval_sec").as_double();
  if (poll_period_ms < 1) {
    poll_period_ms = 1;
  }
  if (reconnect_interval_sec_ < 0.1) {
    reconnect_interval_sec_ = 0.1;
  }
  last_reconnect_attempt_time_ = this->get_clock()->now() - rclcpp::Duration::from_seconds(reconnect_interval_sec_);

  configureSerialPort(serial_port_name_, baud_rate_);

  tx_frame_sub_ = this->create_subscription<std_msgs::msg::String>(
    STM32_TX_FRAME_TOPIC, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&Stm32Bridge::txFrameCallback, this, std::placeholders::_1));
  rx_frame_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
    STM32_RX_FRAME_TOPIC, rclcpp::QoS(rclcpp::KeepLast(20)));

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_period_ms),
    std::bind(&Stm32Bridge::pollSerial, this));

  RCLCPP_INFO(
    get_logger(), "STM32 bridge started: port=%s baud=%d poll_period_ms=%d",
    serial_port_name_.c_str(), baud_rate_, poll_period_ms);
}

void Stm32Bridge::configureSerialPort(const std::string & port, int baud_rate)
{
  try {
    serial_port_.setPort(port);
    serial_port_.setBaudrate(static_cast<uint32_t>(baud_rate));
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setTimeout(timeout);
    serial_port_.open();
    serial_available_ = serial_port_.isOpen();
    if (serial_available_) {
      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port.c_str(), baud_rate);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open serial: %s @ %d", port.c_str(), baud_rate);
    }
  } catch (const std::exception & e) {
    serial_available_ = false;
    RCLCPP_ERROR(
      get_logger(), "Serial open failed for %s @ %d: %s", port.c_str(), baud_rate, e.what());
  }
}

void Stm32Bridge::handleSerialFailure(const std::string & context, const std::string & error_text)
{
  serial_available_ = false;
  try {
    if (serial_port_.isOpen()) {
      serial_port_.close();
    }
  } catch (const std::exception &) {
  }

  RCLCPP_ERROR(get_logger(), "%s: %s", context.c_str(), error_text.c_str());
}

void Stm32Bridge::tryReconnect()
{
  const auto now = this->get_clock()->now();
  if ((now - last_reconnect_attempt_time_).seconds() < reconnect_interval_sec_) {
    return;
  }

  last_reconnect_attempt_time_ = now;
  RCLCPP_WARN(
    get_logger(), "Serial unavailable, attempting reconnect to %s @ %d",
    serial_port_name_.c_str(), baud_rate_);
  configureSerialPort(serial_port_name_, baud_rate_);
}

void Stm32Bridge::txFrameCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!serial_available_ || !serial_port_.isOpen()) {
    tryReconnect();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Serial port unavailable, skip STM32 write");
    return;
  }

  try {
    serial_port_.write(msg->data);
  } catch (const serial::IOException & e) {
    handleSerialFailure("Serial communication error", e.what());
  } catch (const std::exception & e) {
    handleSerialFailure("Serial write failed", e.what());
  }
}

void Stm32Bridge::pollSerial()
{
  if (!serial_available_ || !serial_port_.isOpen()) {
    tryReconnect();
    return;
  }

  try {
    const size_t available = serial_port_.available();
    if (available == 0) {
      return;
    }

    std::string raw = serial_port_.read(available);
    if (raw.empty()) {
      return;
    }

    std_msgs::msg::UInt8MultiArray frame_msg;
    frame_msg.data.reserve(raw.size());
    for (const unsigned char byte : raw) {
      frame_msg.data.push_back(byte);
    }
    rx_frame_pub_->publish(frame_msg);
  } catch (const serial::IOException & e) {
    handleSerialFailure("Serial read error", e.what());
  } catch (const std::exception & e) {
    handleSerialFailure("Serial read failed", e.what());
  }
}

}  // namespace real_vel_transform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_vel_transform::Stm32Bridge>());
  rclcpp::shutdown();
  return 0;
}
