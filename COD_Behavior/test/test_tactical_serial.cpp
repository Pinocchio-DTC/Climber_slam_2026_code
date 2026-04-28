#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>

using namespace std::chrono_literals;

class TacticalSerialPublisher : public rclcpp::Node {
public:
    TacticalSerialPublisher()
        : Node("tactical_serial_publisher") {
        this->declare_parameter<std::string>("scenario", "front_patrol");
        this->declare_parameter<int>("publish_period_ms", 200);

        scenario_ = this->get_parameter("scenario").as_string();
        const auto publish_period_ms = this->get_parameter("publish_period_ms").as_int();

        publisher_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>("SerialReceiveData", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&TacticalSerialPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "test_tactical_serial scenario=%s", scenario_.c_str());
    }

private:
    void fill_common(rm_interfaces::msg::SerialReceiveData &msg) {
        msg.source_mode = "seven";
        msg.match_time = 180;
        msg.enemy_outpost_alive = true;
        msg.our_outpost_alive = true;
        msg.enemy_base_hp = 5000;
        msg.sentry_mode = 0;
        msg.sentry_buff = false;
    }

    void timer_callback() {
        rm_interfaces::msg::SerialReceiveData msg;
        fill_common(msg);

        if (scenario_ == "front_patrol") {
            msg.hp = 450;
            msg.our_base_hp = 5000;
            msg.sentry_buff = true;
        } else if (scenario_ == "low_hp") {
            msg.hp = 200;
            msg.our_base_hp = 5000;
        } else if (scenario_ == "base_low") {
            msg.hp = 450;
            msg.our_base_hp = 3000;
            msg.sentry_buff = true;
        } else if (scenario_ == "recovering") {
            msg.hp = 300;
            msg.our_base_hp = 5000;
        } else {
            msg.hp = 450;
            msg.our_base_hp = 5000;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "unknown scenario [%s], fallback to front_patrol",
                scenario_.c_str());
        }

        publisher_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "publish scenario=%s hp=%u our_base_hp=%u buff=%d",
            scenario_.c_str(),
            msg.hp,
            msg.our_base_hp,
            static_cast<int>(msg.sentry_buff));
    }

    std::string scenario_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TacticalSerialPublisher>());
    rclcpp::shutdown();
    return 0;
}
