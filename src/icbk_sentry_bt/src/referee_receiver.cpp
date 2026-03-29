#include "referee_receiver.h"

#include <algorithm>
#include <chrono>
#include <cstring>

SentryRefereeReceiver::SentryRefereeReceiver() : Node("referee_receiver")
{
    this->declare_parameter("serial_port", "/dev/stm32_robot_base");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("referee_poll_period_ms", 2);

    const std::string port = this->get_parameter("serial_port").as_string();
    const int baud_rate = this->get_parameter("baud_rate").as_int();
    int poll_period_ms = this->get_parameter("referee_poll_period_ms").as_int();
    if (poll_period_ms < 1) {
        poll_period_ms = 1;
    }

    ConfigureSerialPort(port, baud_rate);

    game_type_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_type", 10);
    game_progress_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_progress", 10);
    remain_time_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/time_remaining", 10);
    hp_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/sentry_hp", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(poll_period_ms),
        std::bind(&SentryRefereeReceiver::timerCallback, this));
}

RefereeData SentryRefereeReceiver::getLatestData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_data_;
}

void SentryRefereeReceiver::ConfigureSerialPort(const std::string& port, int baud_rate)
{
    try {
        ser.Open(port);

        LibSerial::BaudRate baud;
        switch (baud_rate) {
            case 9600: baud = LibSerial::BaudRate::BAUD_9600; break;
            case 19200: baud = LibSerial::BaudRate::BAUD_19200; break;
            case 38400: baud = LibSerial::BaudRate::BAUD_38400; break;
            case 57600: baud = LibSerial::BaudRate::BAUD_57600; break;
            case 115200: baud = LibSerial::BaudRate::BAUD_115200; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud_rate);
                return;
        }

        ser.SetBaudRate(baud);
        ser.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        ser.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        ser.SetParity(LibSerial::Parity::PARITY_NONE);
        ser.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

        if (ser.IsOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial opened: %s", port.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s", port.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
    }
}

void SentryRefereeReceiver::timerCallback()
{
    try {
        if (!ser.IsOpen()) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Serial port is not open, skip reading referee data");
            return;
        }

        // Drain serial RX buffer in one callback to keep referee data fresh.
        size_t read_loops = 0;
        constexpr size_t kMaxReadLoopsPerTick = 64;
        while (ser.IsDataAvailable() && read_loops < kMaxReadLoopsPerTick) {
            std::vector<uint8_t> data;
            const size_t bytes_to_read = ser.GetNumberOfBytesAvailable();
            if (bytes_to_read == 0) {
                break;
            }
            ser.Read(data, bytes_to_read);
            if (!data.empty()) {
                buffer.insert(buffer.end(), data.begin(), data.end());
                proccessDataBuffer();
            }
            ++read_loops;
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial read exception: %s", e.what());
    }
}

void SentryRefereeReceiver::proccessDataBuffer()
{
    const uint8_t frame_header = 0xFF;
    const uint8_t frame_tail = 0xFE;
    const size_t frame_size = 20;

    while (true) {
        auto it = std::find(buffer.begin(), buffer.end(), frame_header);
        if (it == buffer.end()) {
            buffer.clear();
            break;
        }

        const size_t frame_start = std::distance(buffer.begin(), it);
        if (frame_start > 0) {
            // Drop leading noise and keep parsing from the first header.
            buffer.erase(buffer.begin(), buffer.begin() + frame_start);
        }

        if (buffer.size() < frame_size) {
            // Keep partial frame for next serial read.
            break;
        }

        if (buffer[frame_size - 1] == frame_tail) {
            parseDataFrame(0);
            buffer.erase(buffer.begin(), buffer.begin() + frame_size);
        } else {
            // Header found but this packet is invalid; shift by one byte and resync.
            buffer.erase(buffer.begin());
        }
    }
}

void SentryRefereeReceiver::parseDataFrame(size_t start_pos)
{
    RefereeData previous_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        previous_data = latest_data_;
    }

    RefereeData parsed_data = previous_data;
    float x = 0.0f, y = 0.0f, angle = 0.0f;
    std::memcpy(&x, &buffer[start_pos + 1], sizeof(float));
    std::memcpy(&y, &buffer[start_pos + 5], sizeof(float));
    std::memcpy(&angle, &buffer[start_pos + 9], sizeof(float));
    parsed_data.x = x;
    parsed_data.y = y;
    parsed_data.angle = angle;
    parsed_data.game_type = buffer[start_pos + 13];
    parsed_data.game_progress = buffer[start_pos + 14];
    parsed_data.time_remaining = (buffer[start_pos + 15] << 8) | buffer[start_pos + 16];
    parsed_data.sentry_hp = (buffer[start_pos + 17] << 8) | buffer[start_pos + 18];
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = parsed_data;
    }

    std_msgs::msg::UInt8 game_type_msg;
    game_type_msg.data = parsed_data.game_type;
    game_type_pub_->publish(game_type_msg);

    std_msgs::msg::UInt8 game_progress_msg;
    game_progress_msg.data = parsed_data.game_progress;
    game_progress_pub_->publish(game_progress_msg);

    std_msgs::msg::UInt16 remain_time_msg;
    remain_time_msg.data = parsed_data.time_remaining;
    remain_time_pub_->publish(remain_time_msg);

    std_msgs::msg::UInt16 hp_msg;
    hp_msg.data = parsed_data.sentry_hp;
    hp_pub_->publish(hp_msg);

    if (parsed_data.sentry_hp != previous_data.sentry_hp ||
        parsed_data.game_progress != previous_data.game_progress ||
        parsed_data.game_type != previous_data.game_type ||
        parsed_data.time_remaining != previous_data.time_remaining) {
        RCLCPP_INFO(
            this->get_logger(),
            "Referee state changed: type=%u progress=%u remain=%u hp=%u",
            static_cast<unsigned>(parsed_data.game_type),
            static_cast<unsigned>(parsed_data.game_progress),
            static_cast<unsigned>(parsed_data.time_remaining),
            static_cast<unsigned>(parsed_data.sentry_hp));
    }

    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Referee frame parsed: pos=(%.3f, %.3f) angle=%.3f type=%u progress=%u remain=%u hp=%u",
        parsed_data.x,
        parsed_data.y,
        parsed_data.angle,
        static_cast<unsigned>(parsed_data.game_type),
        static_cast<unsigned>(parsed_data.game_progress),
        static_cast<unsigned>(parsed_data.time_remaining),
        static_cast<unsigned>(parsed_data.sentry_hp));
}
