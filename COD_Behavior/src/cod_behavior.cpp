//
// Created by ysl on 2025/12/11.
//
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "cod_behavior/action.h"
#include "cod_behavior/condition.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto global_node_ = std::make_shared<rclcpp::Node>(
        "cod_behavior",
        options
    );

    // 配置 RosNodeParams
    BT::RosNodeParams params;
    params.nh = global_node_;
    params.default_port_value = "/navigate_to_pose";
    params.server_timeout = std::chrono::milliseconds(5000);
    params.wait_for_server_timeout = std::chrono::milliseconds(10000);

    // 创建行为树工厂
    BT::BehaviorTreeFactory factory;

    // FollowWaypoints 使用独立的 RosNodeParams，action name 不同
    BT::RosNodeParams follow_wp_params;
    follow_wp_params.nh = global_node_;
    follow_wp_params.default_port_value = "/follow_waypoints";
    follow_wp_params.server_timeout = std::chrono::milliseconds(5000);
    follow_wp_params.wait_for_server_timeout = std::chrono::milliseconds(10000);

    // NavigateThroughPoses 穿越导航
    BT::RosNodeParams nav_through_params;
    nav_through_params.nh = global_node_;
    nav_through_params.default_port_value = "/navigate_through_poses";
    nav_through_params.server_timeout = std::chrono::milliseconds(5000);
    nav_through_params.wait_for_server_timeout = std::chrono::milliseconds(10000);

    // 注册自定义节点（保持你的注册逻辑）

    factory.registerBuilder<SendNav2Goal>(
        "SendNav2Goal",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SendNav2Goal>(name, config, params);
        }
    );

    factory.registerBuilder<FollowWaypointsAction>(
    "FollowWaypointsAction",
    [&](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<FollowWaypointsAction>(name, config, follow_wp_params);
    }
    );
    factory.registerBuilder<NavigateThroughPosesAction>(
        "NavigateThroughPosesAction",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<NavigateThroughPosesAction>(name, config, nav_through_params);
        }
    );

    // PubNav2Goal 使用话题发布导航目标点
    BT::RosNodeParams pub_goal_params;
    pub_goal_params.nh = global_node_;
    pub_goal_params.default_port_value = "/goal_pose";  // Nav2 默认目标点话题

    factory.registerBuilder<PubNav2Goal>(
        "PubNav2Goal",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<PubNav2Goal>(name, config, pub_goal_params);
        }
    );
    factory.registerBuilder<WriteToBlackboard>(
        "WriteToBlackboard",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<WriteToBlackboard>(name, config, global_node_);
        }
    );
    factory.registerNodeType<AnnounceBehavior>("AnnounceBehavior");
    factory.registerBuilder<SetSentryMode>(
        "SetSentryMode",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<SetSentryMode>(name, config, global_node_);
        }
    );

    factory.registerNodeType<HpCondition>("HpCondition");
    factory.registerNodeType<HpAboveCondition>("HpAboveCondition");
    factory.registerNodeType<StayHome>("StayHome");
    factory.registerNodeType<BaseHpBelowCondition>("BaseHpBelowCondition");

    // PubNav2Goal 巡逻方案节点
    factory.registerNodeType<LoadWaypoints>("LoadWaypoints");
    factory.registerNodeType<GetCurrentWaypoint>("GetCurrentWaypoint");
    factory.registerNodeType<NextWaypoint>("NextWaypoint");
    factory.registerNodeType<WaitDuration>("WaitDuration");
    factory.registerBuilder<WaitUntilReached>(
        "WaitUntilReached",
        [&](const std::string &name, const BT::NodeConfig &config) {
            return std::make_unique<WaitUntilReached>(name, config, global_node_);
        }
    );

    const std::string cod_bt = global_node_->get_parameter("cod_bt_path").as_string();

    try {
        auto tree = factory.createTreeFromFile(cod_bt);
        auto blackboard = tree.rootBlackboard();

        // 存入全局节点，增加强引用
        blackboard->set<std::shared_ptr<rclcpp::Node> >("global_node", global_node_);

        // 初始化黑板数据（保持你的逻辑）
        auto homegoal = loadPoseStamped(global_node_, "nav_pose.home");
        auto maingoal =
            global_node_->has_parameter("nav_pose.main.frame_id")
                ? loadPoseStamped(global_node_, "nav_pose.main")
                : homegoal;
        auto maingoal1 =
            global_node_->has_parameter("nav_pose.main1.frame_id")
                ? loadPoseStamped(global_node_, "nav_pose.main1")
                : maingoal;
        auto fortressgoal =
            global_node_->has_parameter("nav_pose.fortress.frame_id")
                ? loadPoseStamped(global_node_, "nav_pose.fortress")
                : homegoal;
        auto dp_goal1 =
            global_node_->has_parameter("D_patrol_pose.first.frame_id")
                ? loadPoseStamped(global_node_, "D_patrol_pose.first")
                : homegoal;
        auto dp_goal2 =
            global_node_->has_parameter("D_patrol_pose.second.frame_id")
                ? loadPoseStamped(global_node_, "D_patrol_pose.second")
                : dp_goal1;
        auto ap_goal1 =
            global_node_->has_parameter("A_patrol_pose.first.frame_id")
                ? loadPoseStamped(global_node_, "A_patrol_pose.first")
                : homegoal;
        auto ap_goal2 =
            global_node_->has_parameter("A_patrol_pose.second.frame_id")
                ? loadPoseStamped(global_node_, "A_patrol_pose.second")
                : ap_goal1;


        blackboard->set<geometry_msgs::msg::PoseStamped>("main_position", maingoal);
        blackboard->set<geometry_msgs::msg::PoseStamped>("main1_position", maingoal1);

        blackboard->set<geometry_msgs::msg::PoseStamped>("home_position", homegoal);
        blackboard->set<geometry_msgs::msg::PoseStamped>("fortress_position", fortressgoal);
        blackboard->set<geometry_msgs::msg::PoseStamped>("dp_position1", dp_goal1);
        blackboard->set<geometry_msgs::msg::PoseStamped>("dp_position2", dp_goal2);
        blackboard->set<geometry_msgs::msg::PoseStamped>("ap_position1", ap_goal1);
        blackboard->set<geometry_msgs::msg::PoseStamped>("ap_position2", ap_goal2);

        blackboard->set<float>("hp", 400.0);
        blackboard->set<bool>("enemy_outpost_alive", false);
        blackboard->set<bool>("our_outpost_alive", false);
        blackboard->set<int>("enemy_base_hp", 0);
        blackboard->set<int>("our_base_hp", 0);
        blackboard->set<int>("sentry_mode", 0);
        blackboard->set<bool>("sentry_buff", false);
        blackboard->set<double>("distance", 0);
        blackboard->set<int>("wp_idx", 0);
        blackboard->set<double>("wait_sec", 5.0);
        blackboard->set<std::string>("current_behavior_label", "");
        blackboard->set<int>("current_sentry_mode", -1);
        blackboard->set<std::string>("current_sentry_mode_name", "");
        blackboard->set<std::string>("desired_sentry_mode_name", "");
        blackboard->set<double>("desired_sentry_mode_started_at", 0.0);
        blackboard->set<bool>("sentry_mode_override_active", false);
        blackboard->set<double>("sentry_mode_override_started_at", 0.0);
        blackboard->set<std::string>("sentry_mode_override_name", "");
        blackboard->set<std::string>(
            "zone_mode_topic",
            global_node_->has_parameter("zone_mode_topic")
                ? global_node_->get_parameter("zone_mode_topic").as_string()
                : "/sentry_mode_cmd");
        blackboard->set<std::string>(
            "front_patrol_csv",
            global_node_->has_parameter("front_patrol_csv")
                ? global_node_->get_parameter("front_patrol_csv").as_string()
                : "");
        blackboard->set<std::string>(
            "fortress_route_csv",
            global_node_->has_parameter("fortress_route_csv")
                ? global_node_->get_parameter("fortress_route_csv").as_string()
                : "");


        const bool enable_groot2 =
            global_node_->has_parameter("enable_groot2")
                ? global_node_->get_parameter("enable_groot2").as_bool()
                : true;
        const int groot2_port =
            global_node_->has_parameter("groot2_port")
                ? static_cast<int>(global_node_->get_parameter("groot2_port").as_int())
                : 5555;

        std::unique_ptr<BT::Groot2Publisher> groot2_publisher;
        if (enable_groot2) {
            try {
                groot2_publisher = std::make_unique<BT::Groot2Publisher>(tree, groot2_port);
                RCLCPP_INFO(global_node_->get_logger(), "Groot2Publisher 已启动，端口: %d", groot2_port);
            } catch (const std::exception &e) {
                RCLCPP_WARN(global_node_->get_logger(),
                            "Groot2Publisher 启动失败，行为树将继续运行: %s", e.what());
            }
        }

        // ========== 核心：单线程主循环（同时处理 ROS 回调 + 行为树） ==========
        while (rclcpp::ok()) {
            // 处理 ROS 待处理的回调（非阻塞，立即返回）
            rclcpp::spin_some(global_node_);
            // 执行一次行为树 tick
            tree.tickOnce();
            // 控制循环频率（避免占用过多 CPU）
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(global_node_->get_logger(), "加载或执行行为树时出错: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
