#ifndef ICBK_SENTRY_BT_REFEREE_RECEIVER_H
#define ICBK_SENTRY_BT_REFEREE_RECEIVER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <libserial/SerialPort.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

struct RefereeData {
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;
    uint8_t game_type = 0;
    uint8_t game_progress = 0;
    uint16_t time_remaining = 0;
    uint16_t sentry_hp = 0;
};

class SentryRefereeReceiver : public rclcpp::Node {
public:
    SentryRefereeReceiver();
    RefereeData getLatestData() const;

private:
    void ConfigureSerialPort(const std::string& port, int baud_rate);
    void timerCallback();
    void proccessDataBuffer();
    void parseDataFrame(size_t start_pos);

    LibSerial::SerialPort ser;
    std::vector<uint8_t> buffer;
    rclcpp::TimerBase::SharedPtr timer_;

    mutable std::mutex data_mutex_;
    RefereeData latest_data_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_type_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_time_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr hp_pub_;
};

#endif  // ICBK_SENTRY_BT_REFEREE_RECEIVER_H
