#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>

using namespace std::chrono_literals;

struct TacticalPhase {
    const char *name;
    uint16_t hp;
    uint16_t our_base_hp;
    bool sentry_buff;
    int duration_sec;
};

class TacticalTransitionPublisher : public rclcpp::Node {
public:
    TacticalTransitionPublisher()
        : Node("tactical_transition_publisher") {
        publisher_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>("SerialReceiveData", 10);
        phases_ = {
            {"front_patrol", 450, 5000, true, 5},
            {"low_hp", 200, 5000, false, 5},
            {"recovering", 300, 5000, false, 5},
            {"front_patrol", 450, 5000, true, 5},
            {"base_low", 450, 3000, true, 5},
        };
        phase_start_time_ = this->now();
        last_phase_name_ = "";

        timer_ = this->create_wall_timer(
            200ms,
            std::bind(&TacticalTransitionPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        update_phase_if_needed();

        const auto &phase = phases_[phase_index_];
        if (last_phase_name_ != phase.name) {
            RCLCPP_INFO(
                this->get_logger(),
                "switch test phase -> %s (hp=%u, our_base_hp=%u, buff=%d)",
                phase.name,
                phase.hp,
                phase.our_base_hp,
                static_cast<int>(phase.sentry_buff));
            last_phase_name_ = phase.name;
        }

        rm_interfaces::msg::SerialReceiveData msg;
        msg.source_mode = "seven";
        msg.match_time = 180;
        msg.hp = phase.hp;
        msg.enemy_outpost_alive = true;
        msg.our_outpost_alive = true;
        msg.enemy_base_hp = 5000;
        msg.our_base_hp = phase.our_base_hp;
        msg.sentry_mode = 0;
        msg.sentry_buff = phase.sentry_buff;
        publisher_->publish(msg);
    }

    void update_phase_if_needed() {
        const auto &phase = phases_[phase_index_];
        const auto elapsed = (this->now() - phase_start_time_).seconds();
        if (elapsed < phase.duration_sec) {
            return;
        }

        phase_index_ = (phase_index_ + 1) % phases_.size();
        phase_start_time_ = this->now();
    }

    std::vector<TacticalPhase> phases_;
    std::size_t phase_index_{0};
    rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
    std::string last_phase_name_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TacticalTransitionPublisher>());
    rclcpp::shutdown();
    return 0;
}
