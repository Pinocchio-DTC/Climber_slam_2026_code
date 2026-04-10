    #include "talker.hpp"

    ReceiveNode::ReceiveNode() : Node("talker"), is_serial_open_(false)
    {
        this->declare_parameter("port_name", "/dev/ttySLAM");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("data_type", DATA_TYPE_SEVEN);

        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        data_type_ = this->get_parameter("data_type").as_string();

        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_port_.setTimeout(timeout);

        // 3. 创建发布者、订阅者、Action 客户端和定时器
        pub_ = this->create_publisher<std_msgs::msg::String>("communication_data", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10, std::bind(&ReceiveNode::cmd_vel_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ReceiveNode::timer_callback, this));

        last_idle_cmd_send_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Node initialized. Port: %s, Mode: %s",
                    port_name_.c_str(), data_type_.c_str());
    }

    // --- 析构函数实现 ---
    ReceiveNode::~ReceiveNode()
    {
        if (is_serial_open_)
            serial_port_.close();
    }

    // --- calc_checksum 实现 ---
    uint8_t ReceiveNode::calc_checksum(const std::vector<uint8_t> &packet)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < packet.size() - 2; ++i)
        {
            sum ^= packet[i];
        }
        return sum;
    }

    void ReceiveNode::send_cmd_packet(int16_t vx, int16_t vy, int16_t vz)
    {
        // 10 字节：帧头 + 3轴速度(6B) + 模式 + 预留1B + 帧尾
        const int CMD_LEN = 10;
        uint8_t send_buff[CMD_LEN] = {0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0xdd};

        send_buff[1] = (uint8_t)((vx >> 8) & 0xFF);
        send_buff[2] = (uint8_t)(vx & 0xFF);
        send_buff[3] = (uint8_t)((vy >> 8) & 0xFF);
        send_buff[4] = (uint8_t)(vy & 0xFF);
        send_buff[5] = (uint8_t)((vz >> 8) & 0xFF);
        send_buff[6] = (uint8_t)(vz & 0xFF);
        send_buff[7] = 0; // 固定模式字节：仅保留串口通信

        try
        {
            serial_port_.write(send_buff, CMD_LEN);
        }
        catch (const std::exception &e)
        {
            (void)e;
        }
    }

    // --- cmd_vel_callback 实现 (发送速度到下位机) ---
    void ReceiveNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cached_vx_ = static_cast<int16_t>(msg->linear.x * 6000);
        cached_vy_ = static_cast<int16_t>(msg->linear.y * 6000);
        cached_vz_ = static_cast<int16_t>(msg->angular.z * 6000);

        if (!is_serial_open_)
            return;

        send_cmd_packet(cached_vx_, cached_vy_, cached_vz_);
    }

    void ReceiveNode::timer_callback()
    {

        if (!is_serial_open_)
        {
            try
            {
                serial_port_.open();
                is_serial_open_ = true;
                RCLCPP_INFO(this->get_logger(), "串口成功打开");
            }
            catch (const serial::IOException &e)
            {
                RCLCPP_WARN(this->get_logger(), "串口打开失败(%s): %s", port_name_.c_str(), e.what());
                return;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Unhandled exception during open: %s", e.what());
                return;
            }
        }

        try
        {
            size_t available = serial_port_.available();
            if (available > 0)
            {
                std::vector<uint8_t> temp(available);
                serial_port_.read(temp.data(), available);
                buffer_.insert(buffer_.end(), temp.begin(), temp.end());
            }

            if (data_type_ == DATA_TYPE_SEVEN)
            {
                parse_seven();
            }
            else
            {
                parse_three();
            }

            // 按 10Hz 周期发送最新速度缓存
            const auto now = this->now();
            if ((now - last_idle_cmd_send_time_).nanoseconds() >= 100000000)
            {
                send_cmd_packet(cached_vx_, cached_vy_, cached_vz_);
                last_idle_cmd_send_time_ = now;
            }
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Serial disconnected: %s", e.what());
            serial_port_.close();
            is_serial_open_ = false;
            buffer_.clear(); // 清空残留数据
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "System error: %s", e.what());
            serial_port_.close();
            is_serial_open_ = false;
        }
    }
    void ReceiveNode::parse_three()
    {

        if (buffer_.size() > 1024)
            buffer_.erase(buffer_.begin(), buffer_.end() - LEN_THREE * 2);

        while (buffer_.size() >= LEN_THREE)
        {
            auto it = std::find(buffer_.begin(), buffer_.end(), FRAME_TAIL);
            if (it == buffer_.end())
                return;

            long tail_idx = std::distance(buffer_.begin(), it);
            long head_idx = tail_idx - (LEN_THREE - 1);

            if (head_idx < 0)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                continue;
            }

            std::vector<uint8_t> packet;
            DownlinkDataThree *data = nullptr;
            bool checksum_ok = false;
            auto erase_end_it = it + 1;

            // 格式A（原定义）：[payload][checksum][tail]
            if (head_idx >= 0)
            {
                packet.assign(buffer_.begin() + head_idx, buffer_.begin() + head_idx + LEN_THREE);
                data = reinterpret_cast<DownlinkDataThree *>(packet.data());
                checksum_ok = (calc_checksum(packet) == data->checksum);
            }

            // 格式B（兼容下位机常见实现）：[payload][tail][checksum]
            if (!checksum_ok)
            {
                long head_idx_b = tail_idx - (LEN_THREE - 2);
                if (head_idx_b >= 0 && (tail_idx + 1) < static_cast<long>(buffer_.size()))
                {
                    packet.assign(buffer_.begin() + head_idx_b, buffer_.begin() + head_idx_b + LEN_THREE);
                    data = reinterpret_cast<DownlinkDataThree *>(packet.data());
                    uint8_t checksum_b = packet[LEN_THREE - 1];
                    checksum_ok = (packet[LEN_THREE - 2] == FRAME_TAIL) && (calc_checksum(packet) == checksum_b);
                    if (checksum_ok)
                    {
                        erase_end_it = it + 2; // 额外吞掉 tail 后面的 checksum
                    }
                }
            }

            if (!checksum_ok)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                RCLCPP_WARN(this->get_logger(), "3V3 Checksum mismatch! Drop packet.");
                continue;
            }

            // 校验成功：移除缓冲区中的这帧数据
            buffer_.erase(buffer_.begin(), erase_end_it);

            // --- 提取并转换全部 12 个数据字段 ---
            int16_t score_diff_val = (int16_t)ntohs(data->score_diff); 
            uint32_t match_time_val = ntohl(data->match_time);
            uint16_t our_hero_blood_val = ntohs(data->our_hero_blood);
            uint16_t our_infantry_blood_val = ntohs(data->our_infantry_blood);
            uint16_t our_sentry_blood_val = ntohs(data->our_sentry_blood);
            uint16_t enemy_hero_blood_val = ntohs(data->enemy_hero_blood);
            uint16_t enemy_infantry_blood_val = ntohs(data->enemy_infantry_blood);
            uint16_t enemy_sentry_blood_val = ntohs(data->enemy_sentry_blood);

            uint8_t our_hero_level_val = data->our_hero_level;
            uint8_t our_infantry_level_val = data->our_infantry_level;
            uint8_t enemy_hero_level_val = data->enemy_hero_level;
            uint8_t enemy_infantry_level_val = data->enemy_infantry_level;

            // 业务逻辑处理 (发布全部消息)
            auto msg = std::make_unique<std_msgs::msg::String>();

            // 拼接全部 12 个字段
            msg->data = "Three|" + to_string(match_time_val) + "|" +
                        to_string(score_diff_val) + "|" +
                        to_string(our_hero_blood_val) + "|" +
                        to_string(our_hero_level_val) + "|" +
                        to_string(our_infantry_blood_val) + "|" +
                        to_string(our_infantry_level_val) + "|" +
                        to_string(our_sentry_blood_val) + "|" +
                        to_string(enemy_hero_blood_val) + "|" +
                        to_string(enemy_hero_level_val) + "|" +
                        to_string(enemy_infantry_blood_val) + "|" +
                        to_string(enemy_infantry_level_val) + "|" +
                        to_string(enemy_sentry_blood_val);

            pub_->publish(std::move(msg));
        }
    }

    // --- parse_seven 实现 ---
    void ReceiveNode::parse_seven()
    {
        // ... (保持原有的实现)
        if (buffer_.size() > 1024)
            buffer_.erase(buffer_.begin(), buffer_.end() - LEN_SEVEN * 2);

        while (buffer_.size() >= LEN_SEVEN)
        {
            auto it = std::find(buffer_.begin(), buffer_.end(), FRAME_TAIL);
            if (it == buffer_.end())
                return;

            long tail_idx = std::distance(buffer_.begin(), it);
            long head_idx = tail_idx - (LEN_SEVEN - 1);

            if (head_idx < 0)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                continue;
            }

            std::vector<uint8_t> packet;
            DownlinkDataSeven *data = nullptr;
            bool checksum_ok = false;
            auto erase_end_it = it + 1;

            // 格式A（原定义）：[payload][checksum][tail]
            if (head_idx >= 0)
            {
                packet.assign(buffer_.begin() + head_idx, buffer_.begin() + head_idx + LEN_SEVEN);
                data = reinterpret_cast<DownlinkDataSeven *>(packet.data());
                checksum_ok = (calc_checksum(packet) == data->checksum);
            }

            // 格式B（兼容）：[payload][tail][checksum]
            if (!checksum_ok)
            {
                long head_idx_b = tail_idx - (LEN_SEVEN - 2);
                if (head_idx_b >= 0 && (tail_idx + 1) < static_cast<long>(buffer_.size()))
                {
                    packet.assign(buffer_.begin() + head_idx_b, buffer_.begin() + head_idx_b + LEN_SEVEN);
                    data = reinterpret_cast<DownlinkDataSeven *>(packet.data());
                    uint8_t checksum_b = packet[LEN_SEVEN - 1];
                    checksum_ok = (packet[LEN_SEVEN - 2] == FRAME_TAIL) && (calc_checksum(packet) == checksum_b);
                    if (checksum_ok)
                    {
                        erase_end_it = it + 2;
                    }
                }
            }

            if (!checksum_ok)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                RCLCPP_WARN(this->get_logger(), "7V7 Checksum mismatch! Drop packet.");
                continue;
            }

            // 校验成功
            buffer_.erase(buffer_.begin(), erase_end_it);

            // --- 提取并转换全部 8 个数据字段 ---
            uint16_t hp_val = ntohs(data->hp);
            uint8_t enemy_outpost_alive_val = data->enemy_outpost_alive;
            uint8_t our_outpost_alive_val = data->our_outpost_alive;
            uint32_t match_time_val = ntohl(data->match_time);
            uint16_t enemy_base_hp_val = ntohs(data->enemy_base_hp);
            uint16_t our_base_hp_val = ntohs(data->our_base_hp);
            uint8_t sentry_mode_val = data->sentry_mode;
            uint8_t sentry_buff_val = data->sentry_buff;

            // 业务逻辑处理 (发布全部消息)
            auto msg = std::make_unique<std_msgs::msg::String>();

            // 拼接全部 8 个字段
            msg->data = "Seven|" + to_string(hp_val) + "|" +
                    to_string(enemy_outpost_alive_val) + "|" +
                    to_string(our_outpost_alive_val) + "|" +
                    to_string(match_time_val) + "|" +
                    to_string(enemy_base_hp_val) + "|" +
                    to_string(our_base_hp_val) + "|" +
                    to_string(sentry_mode_val) + "|" +
                    (sentry_buff_val ? "是" : "否");

            pub_->publish(std::move(msg));
        }
    }

    // --- main 函数 ---
    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        // 引用 ReceiveNode 类，而不是内联定义
        rclcpp::spin(std::make_shared<ReceiveNode>());
        rclcpp::shutdown();          
        return 0;
    }
