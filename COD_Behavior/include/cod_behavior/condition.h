#pragma once
#include "include.h"

//血量检测
class HpCondition : public BT::ConditionNode {
public:
    HpCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("Hp"),
            BT::InputPort<float>("threshold", 210.0f, "hp must be lower than this threshold")
        };
    }

    BT::NodeStatus tick() override {
        auto hp_ = getInput<float>("Hp");
        auto threshold_ = getInput<float>("threshold");

        if (!hp_ || !threshold_) {
            throw BT::RuntimeError(
                "missing input [Hp] or [threshold]"
            );
        }
        const float hp = hp_.value();
        const float threshold = threshold_.value();
        if (hp < threshold) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
};

class StayHome : public BT::ConditionNode {
public:
    StayHome(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("Hp")};
    }

    BT::NodeStatus tick() override {
        auto hp_ = getInput<float>("Hp");

        if (!hp_) {
            throw BT::RuntimeError(
                "missing input [Hp]: ", hp_.error()
            );
        }
        float hp = hp_.value();
        if (hp >= 350) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }
};

class HpAboveCondition : public BT::ConditionNode {
public:
    HpAboveCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<float>("Hp"),
            BT::InputPort<float>("threshold", 400.0f, "hp must be greater than this threshold")
        };
    }

    BT::NodeStatus tick() override {
        auto hp_ = getInput<float>("Hp");
        auto threshold_ = getInput<float>("threshold");

        if (!hp_ || !threshold_) {
            throw BT::RuntimeError(
                "missing input [Hp] or [threshold]"
            );
        }

        const float hp = hp_.value();
        const float threshold = threshold_.value();
        if (hp > threshold) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class BaseHpBelowCondition : public BT::ConditionNode {
public:
    BaseHpBelowCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("Our_base_hp"),
            BT::InputPort<int>("threshold", 3000, "base hp must be lower than or equal to this threshold")
        };
    }

    BT::NodeStatus tick() override {
        auto our_base_hp_ = getInput<int>("Our_base_hp");
        auto threshold_ = getInput<int>("threshold");

        if (!our_base_hp_ || !threshold_) {
            throw BT::RuntimeError(
                "missing input [Our_base_hp] or [threshold]"
            );
        }

        const int our_base_hp = our_base_hp_.value();
        const int threshold = threshold_.value();
        if (our_base_hp <= threshold) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};
