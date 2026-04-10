#pragma once
#include "include.h"

//血量检测
class HpCondition : public BT::ConditionNode {
public:
    HpCondition(const std::string &name, const BT::NodeConfiguration &config)
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
        if (hp < 210) {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
};

class ZsCondition : public BT::ConditionNode {
public:
    ZsCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("Zone_status")};
    }

    BT::NodeStatus tick() override {
        auto zone_status_ = getInput<bool>("Zone_status");

        if (!zone_status_) {
            throw BT::RuntimeError(
                "missing input [Zone_status]: ", zone_status_.error()
            );
        }
        bool zone_status = zone_status_.value();
        if (zone_status) {
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

class DefencePatrolConditioin : public BT::ConditionNode {
public:
    DefencePatrolConditioin(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("Is_defence")};
    }

    BT::NodeStatus tick() override {
        auto is_defence_ = getInput<bool>("Is_defence");

        if (!is_defence_) {
            throw BT::RuntimeError(
                "missing input [Is_defence]: ", is_defence_.error()
            );
        }
        bool is_defence = is_defence_.value();
        if (is_defence) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class AttackPatrolCondition : public BT::ConditionNode {
public:
    AttackPatrolCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("Is_attack")};
    }

    BT::NodeStatus tick() override {
        auto is_attack_ = getInput<bool>("Is_attack");

        if (!is_attack_) {
            throw BT::RuntimeError(
                "missing input [Is_defence]: ", is_attack_.error()
            );
        }
        bool is_attack = is_attack_.value();
        if (is_attack) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class OffensiveCondition : public BT::ConditionNode {
public:
    OffensiveCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("Zone_status"),
            BT::InputPort<bool>("Self_stattus")
        };
    }

    BT::NodeStatus tick() override {
        auto zone_status_ = getInput<bool>("Zone_status");
        auto self_status_ = getInput<bool>("Self_stattus");

        if (!zone_status_ || !self_status_) {
            throw BT::RuntimeError(
                "missing input [Zone_status]: ", zone_status_.error(),
                "missing input [Self_stattus]: ", self_status_.error()
            );
        }
        bool zone_status = zone_status_.value();
        bool self_status = self_status_.value();
        if (zone_status && !self_status) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

class ZonePatrolCondition : public BT::ConditionNode {
public:
    ZonePatrolCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("Zone_status")  
        };
    }

    BT::NodeStatus tick() override {
        auto zone_status_ = getInput<bool>("Zone_status");

        if (!zone_status_) {
            throw BT::RuntimeError(
                "missing input [Zone_status]: ", zone_status_.error()   
            );
        }
        bool zone_status = zone_status_.value();
        if (!zone_status) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};