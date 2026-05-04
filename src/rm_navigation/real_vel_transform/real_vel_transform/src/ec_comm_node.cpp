#include "real_vel_transform/ec_comm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>

namespace real_vel_transform
{

EcCommNode::EcCommNode(const rclcpp::NodeOptions & options)
: Node("ec_comm_node", options),
  serial_available_(false),
  baud_rate_(921600),
  reconnect_interval_sec_(1.0),
  cmd_vel_timeout_sec_(0.3),
  home_arrived_flag_(0),
  home_arrived_state_(false),
  require_go_home_decision_for_arrived_(false),
  has_cmd_vel_(false),
  watchdog_active_(false)
{
  this->declare_parameter<std::string>("serial_port", "/dev/stm32_serial");
  this->declare_parameter<int>("baud_rate", 921600);
  this->declare_parameter<int>("poll_period_ms", 2);
  this->declare_parameter<double>("reconnect_interval_sec", 1.0);
  this->declare_parameter<double>("cmd_vel_timeout_sec", 0.3);
  this->declare_parameter<bool>("ec_home_arrived_require_go_home_decision", false);

  serial_port_name_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  int poll_period_ms = this->get_parameter("poll_period_ms").as_int();
  reconnect_interval_sec_ = this->get_parameter("reconnect_interval_sec").as_double();
  cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();
  require_go_home_decision_for_arrived_ =
    this->get_parameter("ec_home_arrived_require_go_home_decision").as_bool();

  if (poll_period_ms < 1) {
    poll_period_ms = 1;
  }
  if (reconnect_interval_sec_ < 0.1) {
    reconnect_interval_sec_ = 0.1;
  }
  if (cmd_vel_timeout_sec_ < 0.05) {
    cmd_vel_timeout_sec_ = 0.05;
  }

  last_cmd_vel_time_ = this->get_clock()->now();
  last_reconnect_attempt_time_ =
    this->get_clock()->now() - rclcpp::Duration::from_seconds(reconnect_interval_sec_);

  configureSerialPort();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_CHASSIS_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&EcCommNode::cmdVelCallback, this, std::placeholders::_1));

  game_type_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_type", 10);
  game_progress_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_progress", 10);
  remain_time_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/time_remaining", 10);
  hp_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/sentry_hp", 10);
  ally_outpost_hp_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/ally_outpost_hp", 10);
  ec_leave_flag_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/ec_leave_flag", 10);
  ec_home_flag_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/ec_home_flag", 10);
  home_arrived_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/referee/home_arrived_flag", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  std_msgs::msg::UInt8 initial_home_arrived_msg;
  initial_home_arrived_msg.data = home_arrived_flag_;
  home_arrived_pub_->publish(initial_home_arrived_msg);

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_period_ms), std::bind(&EcCommNode::pollSerial, this));
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&EcCommNode::watchdogCallback, this));

  RCLCPP_INFO(
    get_logger(), "EC comm started: port=%s baud=%d poll_period_ms=%d",
    serial_port_name_.c_str(), baud_rate_, poll_period_ms);
}

void EcCommNode::configureSerialPort()
{
  try {
    serial_port_.setPort(serial_port_name_);
    serial_port_.setBaudrate(static_cast<uint32_t>(baud_rate_));
    serial_port_.setFlowcontrol(serial::flowcontrol_none);
    serial_port_.setParity(serial::parity_none);
    serial_port_.setStopbits(serial::stopbits_one);
    serial_port_.setBytesize(serial::eightbits);
    serial_port_.setTimeout(serial::Timeout::simpleTimeout(2));
    serial_port_.open();
    serial_available_ = serial_port_.isOpen();
    if (serial_available_) {
      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", serial_port_name_.c_str(), baud_rate_);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open serial: %s @ %d", serial_port_name_.c_str(), baud_rate_);
    }
  } catch (const std::exception & e) {
    serial_available_ = false;
    RCLCPP_ERROR(
      get_logger(), "Serial open failed for %s @ %d: %s",
      serial_port_name_.c_str(), baud_rate_, e.what());
  }
}

void EcCommNode::closeSerialPort()
{
  try {
    if (serial_port_.isOpen()) {
      serial_port_.close();
    }
  } catch (const std::exception &) {
  }
  serial_available_ = false;
}

void EcCommNode::handleSerialFailure(const std::string & context, const std::string & error_text)
{
  closeSerialPort();
  RCLCPP_ERROR(get_logger(), "%s: %s", context.c_str(), error_text.c_str());
}

void EcCommNode::tryReconnect()
{
  const auto now = this->get_clock()->now();
  if ((now - last_reconnect_attempt_time_).seconds() < reconnect_interval_sec_) {
    return;
  }

  last_reconnect_attempt_time_ = now;
  RCLCPP_WARN(
    get_logger(), "Serial unavailable, attempting reconnect to %s @ %d",
    serial_port_name_.c_str(), baud_rate_);
  configureSerialPort();
}

void EcCommNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  has_cmd_vel_ = true;
  watchdog_active_ = false;
  last_cmd_vel_time_ = this->get_clock()->now();
  sendVelocityFrame(*msg);
}

void EcCommNode::watchdogCallback()
{
  if (!has_cmd_vel_) {
    return;
  }

  const double age = (this->get_clock()->now() - last_cmd_vel_time_).seconds();
  if (age <= cmd_vel_timeout_sec_) {
    return;
  }

  geometry_msgs::msg::Twist stop_msg;
  sendVelocityFrame(stop_msg);

  if (!watchdog_active_) {
    RCLCPP_WARN(
      get_logger(), "EC comm watchdog timeout: cmd_vel_chassis stale for %.3f s, forcing stop",
      age);
  }
  watchdog_active_ = true;
}

void EcCommNode::sendVelocityFrame(const geometry_msgs::msg::Twist & msg)
{
  if (!serial_available_ || !serial_port_.isOpen()) {
    tryReconnect();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial unavailable, skip velocity frame");
    return;
  }

  NavToEcFrame frame;
  frame.vx = static_cast<float>(msg.linear.x);
  frame.vy = static_cast<float>(msg.linear.y);
  frame.wz = static_cast<float>(msg.angular.z);
  frame.home_arrived_flag = home_arrived_flag_;
  frame.crc16 = calculateCrc16(
    reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - sizeof(frame.crc16));

  try {
    serial_port_.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
  } catch (const serial::IOException & e) {
    handleSerialFailure("Serial write error", e.what());
  } catch (const std::exception & e) {
    handleSerialFailure("Serial write failed", e.what());
  }
}

void EcCommNode::pollSerial()
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

    std::vector<uint8_t> bytes(available);
    const size_t read_count = serial_port_.read(bytes.data(), bytes.size());
    bytes.resize(read_count);
    rx_buffer_.insert(rx_buffer_.end(), bytes.begin(), bytes.end());
    processRxBuffer();
  } catch (const serial::IOException & e) {
    handleSerialFailure("Serial read error", e.what());
  } catch (const std::exception & e) {
    handleSerialFailure("Serial read failed", e.what());
  }
}

void EcCommNode::processRxBuffer()
{
  constexpr size_t kFrameSize = sizeof(EcToNavFrame);

  while (true) {
    auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), static_cast<uint8_t>('S'));
    while (it != rx_buffer_.end()) {
      const auto next = it + 1;
      if (next != rx_buffer_.end() && *next == static_cast<uint8_t>('P')) {
        break;
      }
      it = std::find(next, rx_buffer_.end(), static_cast<uint8_t>('S'));
    }

    if (it == rx_buffer_.end()) {
      if (!rx_buffer_.empty() && rx_buffer_.back() == static_cast<uint8_t>('S')) {
        rx_buffer_.assign(1, static_cast<uint8_t>('S'));
        return;
      }
      rx_buffer_.clear();
      return;
    }

    if (it != rx_buffer_.begin()) {
      rx_buffer_.erase(rx_buffer_.begin(), it);
    }

    if (rx_buffer_.size() < kFrameSize) {
      return;
    }

    EcToNavFrame frame;
    std::memcpy(&frame, rx_buffer_.data(), sizeof(frame));
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kFrameSize);

    if (!checkCrc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Drop EC frame with bad CRC16");
      continue;
    }

    publishEcFrame(frame);
  }
}

void EcCommNode::publishEcFrame(const EcToNavFrame & frame)
{
  if (frame.game_type > 10 || frame.game_progress > 10 ||
    frame.ec_leave_flag > 1 || frame.ec_home_flag > 1)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Drop invalid EC frame: type=%u progress=%u leave=%u home=%u",
      static_cast<unsigned>(frame.game_type), static_cast<unsigned>(frame.game_progress),
      static_cast<unsigned>(frame.ec_leave_flag), static_cast<unsigned>(frame.ec_home_flag));
    return;
  }

  std_msgs::msg::UInt8 u8_msg;
  std_msgs::msg::UInt16 u16_msg;

  u8_msg.data = frame.game_type;
  game_type_pub_->publish(u8_msg);
  u8_msg.data = frame.game_progress;
  game_progress_pub_->publish(u8_msg);

  u16_msg.data = frame.time_remaining;
  remain_time_pub_->publish(u16_msg);
  u16_msg.data = frame.sentry_hp;
  hp_pub_->publish(u16_msg);
  u16_msg.data = frame.ally_outpost_hp;
  ally_outpost_hp_pub_->publish(u16_msg);

  u8_msg.data = frame.ec_leave_flag;
  ec_leave_flag_pub_->publish(u8_msg);
  u8_msg.data = frame.ec_home_flag;
  ec_home_flag_pub_->publish(u8_msg);

  bool arrived_home = frame.ec_home_flag == 1;
  if (require_go_home_decision_for_arrived_) {
    arrived_home = arrived_home && frame.ec_leave_flag == 0;
  }

  if (arrived_home != home_arrived_state_) {
    home_arrived_state_ = arrived_home;
    home_arrived_flag_ = arrived_home ? 1 : 0;
    u8_msg.data = home_arrived_flag_;
    home_arrived_pub_->publish(u8_msg);
  }
}

uint16_t EcCommNode::calculateCrc16(const uint8_t * data, size_t length) const
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

bool EcCommNode::checkCrc16(const uint8_t * data, size_t length) const
{
  if (length < sizeof(uint16_t)) {
    return false;
  }

  uint16_t received_crc = 0;
  std::memcpy(&received_crc, data + length - sizeof(received_crc), sizeof(received_crc));
  return received_crc == calculateCrc16(data, length - sizeof(received_crc));
}

}  // namespace real_vel_transform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_vel_transform::EcCommNode>());
  rclcpp::shutdown();
  return 0;
}
