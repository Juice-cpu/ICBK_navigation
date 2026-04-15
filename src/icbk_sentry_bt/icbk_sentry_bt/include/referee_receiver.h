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
    // 己方前哨站血量，为 0 时可触发登堡垒策略。
    uint16_t ally_outpost_hp = 0;
    // 电控回传的原始 0/1 标志位，只有值严格等于 1 才视为有效。
    uint8_t ec_leave_flag = 0;
    uint8_t ec_home_flag = 0;
};

class SentryRefereeReceiver : public rclcpp::Node {
public:
    SentryRefereeReceiver();
    RefereeData getLatestData() const;

private:
    void ConfigureSerialPort(const std::string& port, int baud_rate);
    void timerCallback();
    void proccessDataBuffer();
    void parseDataFrame(size_t start_pos, size_t frame_size);

    LibSerial::SerialPort ser;
    std::vector<uint8_t> buffer;
    rclcpp::TimerBase::SharedPtr timer_;

    mutable std::mutex data_mutex_;
    RefereeData latest_data_;

    uint64_t total_bytes_received_ = 0;
    uint64_t total_frames_parsed_ = 0;
    rclcpp::Time last_byte_time_;
    rclcpp::Time last_frame_time_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_type_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_time_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr hp_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr ally_outpost_hp_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr home_arrived_pub_;
    double home_point_x_ = 0.0;
    double home_point_y_ = 0.0;
    double home_arrived_radius_ = 0.35;
    bool home_arrived_state_ = false;
    bool ec_home_arrived_require_go_home_decision_ = false;
};

#endif  // ICBK_SENTRY_BT_REFEREE_RECEIVER_H
