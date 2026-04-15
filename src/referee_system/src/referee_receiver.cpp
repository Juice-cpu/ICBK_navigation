#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <vector>
#include <cstring>
#include <stdint.h>
#include <iostream>

/**
 * @class SentryRefereeReceiver
 * @brief 裁判系统数据接收节点类
 * 
 * 该节点负责通过串口接收来自裁判系统的数据，
 * 解析数据帧并提取机器人位置、角度和血量等信息。
 */

class SentryRefereeReceiver : public rclcpp::Node {
public:
    SentryRefereeReceiver() : Node("referee_receiver") {
        // 声明串口参数
        this->declare_parameter("serial_port", "/dev/ACM0"); // 虚拟串口
        this->declare_parameter("baud_rate", 115200);  // 波特率
        
        // 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        
        // 配置串口
        ConfigureSerialPort(port, baud_rate);
        
        // 创建定时器，定期读取串口数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SentryRefereeReceiver::timerCallback, this));
    }

private:
    void ConfigureSerialPort(const std::string& port, int baud_rate) {
        try {
            ser.setPort(port); //设置串口设备
            ser.setBaudrate(baud_rate); //设置波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            
            if(ser.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "串口已连接： %s", port.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "无法打开串口： %s", port.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "串口异常： %s", e.what());
        }
    }

    void timerCallback() {
        //检查串口是否有可用数据
        if(ser.available()) {
            std::vector<uint8_t> data;
            // 读取所有可用数据
            ser.read(data, ser.available());
            // 将新数据添加到缓冲区末尾
            buffer.insert(buffer.end(), data.begin(), data.end());
            proccessDataBuffer();//处理缓冲区中的数据
        }
    }

    void proccessDataBuffer() {
        // 定义数据帧帧头、帧尾和帧长度
        const uint8_t FRAME_HEADER = 0xFF; //帧头标记
        const uint8_t FRAME_TAIL = 0xFE; //帧尾标记
        const size_t FRAME_SIZE = 20; // 电控发送的帧长度是20字节
        
        // 当缓冲区数据足够一个完整帧时，进行处理
        while(buffer.size() >= FRAME_SIZE) {
            // 查找帧头位置
            auto it = std::find(buffer.begin(), buffer.end(), FRAME_HEADER);
            if(it == buffer.end()) {
                // 未找到帧头，清空缓冲区
                buffer.clear();
                break;
            }
            
            // 计算帧头位置索引
            size_t frame_start = std::distance(buffer.begin(), it);
            // 检查是否有完整帧，并且帧尾正确
            if(frame_start + FRAME_SIZE <= buffer.size() && buffer[frame_start + FRAME_SIZE - 1] == FRAME_TAIL) {
                // 解析数据帧
                parseDataFrame(frame_start);
                // 移除已处理的帧数据
                buffer.erase(buffer.begin(), buffer.begin() + frame_start + FRAME_SIZE);
            } else {
                // 帧不完整或帧尾错误，移除无效数据
                buffer.erase(buffer.begin(), buffer.begin() + frame_start + 1);
            }
        }
    }

    void parseDataFrame(size_t start_pos) {
        // 解析位置数据（每个浮点数占4字节）
        float x = *(reinterpret_cast<float*>(&buffer[start_pos + 1]));
        float y = *(reinterpret_cast<float*>(&buffer[start_pos + 5]));
        
        // 解析角度数据
        float angle = *(reinterpret_cast<float*>(&buffer[start_pos + 9]));
        
        // 解析比赛类型
        uint8_t game_type = buffer[start_pos + 13];
        
        // 解析比赛阶段
        uint8_t game_progress = buffer[start_pos + 14];
        
        // 解析比赛剩余时间（2字节）
        uint16_t time_remaining = (buffer[start_pos + 15] << 8) | buffer[start_pos + 16];
        
        // 解析机器人血量（2字节）
        uint16_t sentry_hp = (buffer[start_pos + 17] << 8) | buffer[start_pos + 18];
        
        // 打印解析结果
        RCLCPP_INFO(this->get_logger(), 
                   "位置: (%f, %f), 角度: %f, 游戏类型: %d, 阶段: %d, 剩余时间: %d, 血量: %d", 
                   x, y, angle, game_type, game_progress, time_remaining, sentry_hp);
    }

    serial::Serial ser; // 串口对象
    std::vector<uint8_t> buffer; // 数据缓冲区
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryRefereeReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
