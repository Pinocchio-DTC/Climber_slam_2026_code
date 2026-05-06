#ifndef TALKER_HPP
#define TALKER_HPP

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

// --- 常量/宏定义 ---
#define FRAME_TAIL 0xDD
#define DATA_TYPE_SEVEN "seven"
#define DATA_TYPE_THREE "three"

// --- 3V3 结构体定义 (24 bytes) ---
#pragma pack(1)
struct DownlinkDataThree
{
    uint32_t match_time;
    int16_t score_diff;
    uint16_t our_hero_blood;
    uint8_t our_hero_level;
    uint16_t our_infantry_blood;
    uint8_t our_infantry_level;
    uint16_t our_sentry_blood;
    uint16_t enemy_hero_blood;
    uint8_t enemy_hero_level;
    uint16_t enemy_infantry_blood;
    uint8_t enemy_infantry_level;
    uint16_t enemy_sentry_blood;
    uint8_t checksum;
    uint8_t frame_tail;
};
#pragma pack()
#define LEN_THREE sizeof(DownlinkDataThree)

// --- 7V7 结构体定义 (16 bytes) ---
#pragma pack(1)
struct DownlinkDataSeven
{
    uint16_t hp;
    uint8_t enemy_outpost_alive;
    uint8_t our_outpost_alive;
    uint32_t match_time;
    uint16_t enemy_base_hp;
    uint16_t our_base_hp;
    uint8_t sentry_mode;
    uint8_t sentry_buff;
    uint8_t checksum;
    uint8_t frame_tail;
};
#pragma pack()
#define LEN_SEVEN sizeof(DownlinkDataSeven)

// --- 类声明 ---
class ReceiveNode : public rclcpp::Node
{
public:
    ReceiveNode();
    ~ReceiveNode();

private:
    // ROS2 成员
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sentry_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_label_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr pub_;

    // 串口通信成员
    serial::Serial serial_port_;
    std::string port_name_;
    int baud_rate_;
    std::string data_type_;
    bool enable_downlink_receive_;
    bool is_serial_open_;
    std::deque<uint8_t> buffer_;

    // 发送到底盘的最新速度缓存（按10Hz周期发送）
    int16_t cached_vx_{0};
    int16_t cached_vy_{0};
    int16_t cached_vz_{0};
    uint8_t cached_sentry_mode_{0};
    rclcpp::Time last_idle_cmd_send_time_;
    rclcpp::Time last_comm_log_time_;
    std::string current_behavior_label_{"unknown"};

    // 最近一次接收到的下行数据摘要，用于周期打印
    std::string last_received_summary_{"no downlink yet"};
    std::string last_sent_summary_{"tx speed[vx=0 vy=0 vz=0 mode=0]"};

    // --- 方法声明 ---

    // 回调函数
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void sentry_mode_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void behavior_label_callback(const std_msgs::msg::String::SharedPtr msg);

    // 解析逻辑
    void parse_three();
    void parse_seven();
    void maybe_log_comm_state();
    std::string build_sent_summary(int16_t vx, int16_t vy, int16_t vz) const;

    // 辅助函数
    uint8_t calc_checksum(const std::vector<uint8_t> &packet);
    void send_cmd_packet(int16_t vx, int16_t vy, int16_t vz);
};

#endif 
