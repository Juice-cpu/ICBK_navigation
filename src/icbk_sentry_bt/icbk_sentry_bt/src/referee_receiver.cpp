#include "referee_receiver.h"

#include <algorithm>
#include <chrono>
#include <cstring>

SentryRefereeReceiver::SentryRefereeReceiver() : Node("referee_receiver")
{
    this->declare_parameter("serial_port", "/dev/stm32_serial");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("referee_poll_period_ms", 2);
    this->declare_parameter("home_point_x", 0.0);
    this->declare_parameter("home_point_y", 0.0);
    this->declare_parameter("home_arrived_radius", 0.35);
    this->declare_parameter("ec_home_arrived_require_go_home_decision", false);

    const std::string port = this->get_parameter("serial_port").as_string();
    const int baud_rate = this->get_parameter("baud_rate").as_int();
    int poll_period_ms = this->get_parameter("referee_poll_period_ms").as_int();
    home_point_x_ = this->get_parameter("home_point_x").as_double();
    home_point_y_ = this->get_parameter("home_point_y").as_double();
    home_arrived_radius_ = this->get_parameter("home_arrived_radius").as_double();
    if (home_arrived_radius_ < 0.05) {
        home_arrived_radius_ = 0.05;
    }
    ec_home_arrived_require_go_home_decision_ =
        this->get_parameter("ec_home_arrived_require_go_home_decision").as_bool();
    if (poll_period_ms < 1) {
        poll_period_ms = 1;
    }

    last_byte_time_ = this->get_clock()->now();
    last_frame_time_ = this->get_clock()->now();

    ConfigureSerialPort(port, baud_rate);

    game_type_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_type", 10);
    game_progress_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/game_progress", 10);
    remain_time_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/time_remaining", 10);
    hp_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/sentry_hp", 10);
    ally_outpost_hp_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/referee/ally_outpost_hp", 10);
    home_arrived_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/referee/home_arrived_flag",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    RCLCPP_INFO(
        this->get_logger(),
        "Home-arrived judge: home=(%.2f, %.2f) radius=%.2f require_go_home_decision=%u",
        home_point_x_,
        home_point_y_,
        home_arrived_radius_,
        static_cast<unsigned>(ec_home_arrived_require_go_home_decision_));
    RCLCPP_INFO(
        this->get_logger(),
        "Referee receiver config: port=%s baud=%d poll_period_ms=%d",
        port.c_str(), baud_rate, poll_period_ms);

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

        // 单次回调尽量把串口缓冲区读空，减少状态刷新延迟。
        size_t read_loops = 0;
        constexpr size_t kMaxReadLoopsPerTick = 64;
        while (ser.IsDataAvailable() && read_loops < kMaxReadLoopsPerTick) {
            const size_t bytes_to_read = ser.GetNumberOfBytesAvailable();
            if (bytes_to_read == 0) {
                break;
            }

            std::vector<uint8_t> data;
            ser.Read(data, bytes_to_read);
            if (!data.empty()) {
                total_bytes_received_ += data.size();
                last_byte_time_ = this->get_clock()->now();
                buffer.insert(buffer.end(), data.begin(), data.end());
                proccessDataBuffer();
            }
            ++read_loops;
        }

        const auto now = this->get_clock()->now();
        const double since_last_byte = (now - last_byte_time_).seconds();
        const double since_last_frame = (now - last_frame_time_).seconds();
        if (total_bytes_received_ == 0 || since_last_byte > 1.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "No referee bytes received recently: total_bytes=%llu since_last_byte=%.3f s",
                static_cast<unsigned long long>(total_bytes_received_), since_last_byte);
        } else if (since_last_frame > 1.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Referee bytes received but no valid frame parsed recently: total_bytes=%llu total_frames=%llu buffered=%zu since_last_frame=%.3f s",
                static_cast<unsigned long long>(total_bytes_received_),
                static_cast<unsigned long long>(total_frames_parsed_),
                buffer.size(),
                since_last_frame);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial read exception: %s", e.what());
    }
}

void SentryRefereeReceiver::proccessDataBuffer()
{
    const uint8_t frame_header = 0xFF;
    const uint8_t frame_tail = 0xFE;
    constexpr size_t kMinFrameSize = 20;
    constexpr size_t kMaxFrameSize = 24;
    constexpr size_t kSupportedFrameSizes[] = {24, 23, 22, 20};

    while (true) {
        auto it = std::find(buffer.begin(), buffer.end(), frame_header);
        if (it == buffer.end()) {
            buffer.clear();
            break;
        }

        const size_t frame_start = std::distance(buffer.begin(), it);
        if (frame_start > 0) {
            // 丢掉帧头之前的噪声字节，从第一个有效帧头重新同步。
            buffer.erase(buffer.begin(), buffer.begin() + frame_start);
        }

        if (buffer.size() < kMinFrameSize) {
            // 数据长度还不够组成最短一帧，等待后续字节补齐。
            break;
        }

        size_t detected_frame_size = 0;
        for (const size_t candidate_size : kSupportedFrameSizes) {
            if (buffer.size() < candidate_size) {
                continue;
            }
            if (buffer[candidate_size - 1] == frame_tail) {
                detected_frame_size = candidate_size;
                break;
            }
        }

        if (detected_frame_size > 0) {
            parseDataFrame(0, detected_frame_size);
            buffer.erase(buffer.begin(), buffer.begin() + detected_frame_size);
            continue;
        }

        if (buffer.size() > kMaxFrameSize) {
            // 超过最大帧长仍未找到帧尾，说明当前同步失败，左移一字节继续搜索。
            buffer.erase(buffer.begin());
            continue;
        }

        break;
    }
}

void SentryRefereeReceiver::parseDataFrame(size_t start_pos, size_t frame_size)
{
    RefereeData previous_data;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        previous_data = latest_data_;
    }

    RefereeData parsed_data = previous_data;
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;
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

    if (frame_size >= 22) {
        parsed_data.ally_outpost_hp =
            (buffer[start_pos + 19] << 8) | buffer[start_pos + 20];
    }
    if (frame_size >= 23) {
        // 电控回传的是原始 0/1 数值，这里保留原值，后续逻辑只对 1 生效。
        parsed_data.ec_leave_flag = buffer[start_pos + 21];
    } else {
        parsed_data.ec_leave_flag = 0;
    }
    if (frame_size >= 24) {
        // 电控回传的是原始 0/1 数值，这里保留原值，后续逻辑只对 1 生效。
        parsed_data.ec_home_flag = buffer[start_pos + 22];
    } else {
        parsed_data.ec_home_flag = 0;
    }

    // 做一层基础合法性过滤，避免错帧把行为树状态直接污染掉。
    const bool game_type_ok = parsed_data.game_type <= 10;
    const bool game_progress_ok = parsed_data.game_progress <= 10;
    const bool leave_flag_ok = parsed_data.ec_leave_flag <= 1;
    const bool home_flag_ok = parsed_data.ec_home_flag <= 1;
    if (!game_type_ok || !game_progress_ok || !leave_flag_ok || !home_flag_ok) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Drop invalid referee frame: size=%zu type=%u progress=%u remain=%u hp=%u outpost_hp=%u leave_flag=%u home_flag=%u",
            frame_size,
            static_cast<unsigned>(parsed_data.game_type),
            static_cast<unsigned>(parsed_data.game_progress),
            static_cast<unsigned>(parsed_data.time_remaining),
            static_cast<unsigned>(parsed_data.sentry_hp),
            static_cast<unsigned>(parsed_data.ally_outpost_hp),
            static_cast<unsigned>(parsed_data.ec_leave_flag),
            static_cast<unsigned>(parsed_data.ec_home_flag));
        return;
    }

    // 只有严格等于 1 才认为电控要求回家/离家，避免异常非零值误触发。
    const bool arrived_home_flag = (parsed_data.ec_home_flag == 1);
    const bool leave_home_flag = (parsed_data.ec_leave_flag == 1);
    bool arrived_home = arrived_home_flag;
    if (ec_home_arrived_require_go_home_decision_) {
        arrived_home = arrived_home && !leave_home_flag;
    }
    if (arrived_home != home_arrived_state_) {
        home_arrived_state_ = arrived_home;
        std_msgs::msg::UInt8 home_arrived_msg;
        home_arrived_msg.data = arrived_home ? 1 : 0;
        home_arrived_pub_->publish(home_arrived_msg);
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = parsed_data;
    }
    ++total_frames_parsed_;
    last_frame_time_ = this->get_clock()->now();

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

    std_msgs::msg::UInt16 ally_outpost_hp_msg;
    ally_outpost_hp_msg.data = parsed_data.ally_outpost_hp;
    ally_outpost_hp_pub_->publish(ally_outpost_hp_msg);

    if (parsed_data.sentry_hp != previous_data.sentry_hp ||
        parsed_data.ally_outpost_hp != previous_data.ally_outpost_hp ||
        parsed_data.game_progress != previous_data.game_progress ||
        parsed_data.game_type != previous_data.game_type ||
        parsed_data.time_remaining != previous_data.time_remaining ||
        parsed_data.ec_leave_flag != previous_data.ec_leave_flag ||
        parsed_data.ec_home_flag != previous_data.ec_home_flag) {
        RCLCPP_INFO(
            this->get_logger(),
            "Referee state changed: type=%u progress=%u remain=%u hp=%u outpost_hp=%u leave_flag=%u home_flag=%u",
            static_cast<unsigned>(parsed_data.game_type),
            static_cast<unsigned>(parsed_data.game_progress),
            static_cast<unsigned>(parsed_data.time_remaining),
            static_cast<unsigned>(parsed_data.sentry_hp),
            static_cast<unsigned>(parsed_data.ally_outpost_hp),
            static_cast<unsigned>(parsed_data.ec_leave_flag),
            static_cast<unsigned>(parsed_data.ec_home_flag));
    }

    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Referee frame parsed: pos=(%.3f, %.3f) angle=%.3f type=%u progress=%u remain=%u hp=%u outpost_hp=%u leave_flag=%u home_flag=%u frame_size=%zu",
        parsed_data.x,
        parsed_data.y,
        parsed_data.angle,
        static_cast<unsigned>(parsed_data.game_type),
        static_cast<unsigned>(parsed_data.game_progress),
        static_cast<unsigned>(parsed_data.time_remaining),
        static_cast<unsigned>(parsed_data.sentry_hp),
        static_cast<unsigned>(parsed_data.ally_outpost_hp),
        static_cast<unsigned>(parsed_data.ec_leave_flag),
        static_cast<unsigned>(parsed_data.ec_home_flag),
        frame_size);
}
