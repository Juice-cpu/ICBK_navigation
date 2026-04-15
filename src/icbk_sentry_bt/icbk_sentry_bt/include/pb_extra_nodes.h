#ifndef ICBK_SENTRY_BT_PB_EXTRA_NODES_H
#define ICBK_SENTRY_BT_PB_EXTRA_NODES_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "referee_receiver.h"
#include "rm_interfaces/msg/target.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// 行为树共享缓存：裁判系统数据源与目标检测缓存都放在这里集中管理。
// Shared BT cache for referee data and the latest visual target.
inline SentryRefereeReceiver* g_referee_source = nullptr;
inline rclcpp::Node::SharedPtr g_target_node = nullptr;
inline rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr g_target_sub = nullptr;
inline rm_interfaces::msg::Target g_last_target{};
inline rclcpp::Time g_last_target_time{};
inline bool g_has_target = false;
inline std::string g_target_topic;

inline rclcpp::Node::SharedPtr make_isolated_node(const std::string& name)
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    return rclcpp::Node::make_shared(name, options);
}

inline void setRefereeSource(SentryRefereeReceiver* ptr)
{
    g_referee_source = ptr;
}

inline SentryRefereeReceiver* resolve_referee_from_port(
    const BT::TreeNode& node,
    const char* port_name = "key_port")
{
    SentryRefereeReceiver* ref_ptr = nullptr;
    try {
        auto ref = node.getInput<SentryRefereeReceiver*>(port_name);
        if (ref) {
            ref_ptr = *ref;
        }
    } catch (const std::exception&) {
    }

    if (!ref_ptr) {
        ref_ptr = g_referee_source;
    }
    if (!ref_ptr) {
        throw BT::RuntimeError("missing required input [key_port]: referee source is not set");
    }
    return ref_ptr;
}

inline void ensureTargetSubscription(const std::string& topic)
{
    if (!g_target_node) {
        // 为行为树内部的目标缓存创建独立节点，避免和主节点回调混用。
        g_target_node = make_isolated_node("bt_target_cache");
    }
    if (g_target_topic != topic || !g_target_sub) {
        g_target_topic = topic;
        g_has_target = false;
        g_target_sub = g_target_node->create_subscription<rm_interfaces::msg::Target>(
            topic, rclcpp::SensorDataQoS(),
            [](const rm_interfaces::msg::Target::SharedPtr msg) {
                g_last_target = *msg;
                g_last_target_time = g_target_node->now();
                g_has_target = true;
            });
    }
}

class IsGameStatus : public BT::ConditionNode {
public:
    IsGameStatus(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
            BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
            BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();

        const int max_remain = getInput<int>("max_remain_time").value();
        const int min_remain = getInput<int>("min_remain_time").value();
        const int expected_progress = getInput<int>("expected_game_progress").value();

        const bool in_time_window = data.time_remaining <= max_remain && data.time_remaining >= min_remain;
        const bool progress_ok = static_cast<int>(data.game_progress) == expected_progress;
        const auto result =
            (in_time_window && progress_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

        return result;
    }
};

class IsStatusOK : public BT::ConditionNode {
public:
    IsStatusOK(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("ammo_min", 0, "Lower than minimum ammo returns FAILURE"),
            BT::InputPort<int>("heat_max", 350, "Maximum heat"),
            BT::InputPort<int>("hp_min", 300, "Minimum HP"),
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();
        const int hp_min = getInput<int>("hp_min").value();
        const auto result =
            (data.sentry_hp >= hp_min) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

        return result;
    }
};

class LowHpLatched : public BT::ConditionNode {
public:
    LowHpLatched(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("enter_hp", 200, "Enter return-home mode below this HP"),
            BT::InputPort<int>("exit_hp", 220, "Exit return-home mode above this HP"),
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();
        const int enter_hp = getInput<int>("enter_hp").value();
        const int exit_hp = getInput<int>("exit_hp").value();

        // 用滞回避免血量在阈值边缘来回抖动，导致行为树频繁切分支。
        if (!latched_ && data.sentry_hp < enter_hp) {
            latched_ = true;
        } else if (latched_ && data.sentry_hp > exit_hp) {
            latched_ = false;
        }

        return latched_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static void reset()
    {
        latched_ = false;
    }

private:
    inline static bool latched_ = false;
};

class IsAllyOutpostDestroyed : public BT::ConditionNode {
public:
    IsAllyOutpostDestroyed(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();
        // 只有己方前哨站血量严格等于 0，才触发登堡垒分支。
        return (data.ally_outpost_hp == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static void resetCache()
    {
        // 无状态节点，保留统一清理入口。
    }
};

class IsEcDecisionGoOut : public BT::ConditionNode {
public:
    IsEcDecisionGoOut(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();
        // 电控标志只有值等于 1 才视为“允许出家”。
        return (data.ec_leave_flag == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static void resetCache()
    {
        // 无状态节点，保留统一清理入口。
    }
};

class IsEcDecisionGoHome : public BT::ConditionNode {
public:
    IsEcDecisionGoHome(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        auto* ref_ptr = resolve_referee_from_port(*this);
        const auto data = ref_ptr->getLatestData();
        // 电控标志只有值等于 1 才视为“要求回家”。
        return (data.ec_home_flag == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static void resetCache()
    {
        // 无状态节点，保留统一清理入口。
    }
};

class IsRfidDetected : public BT::ConditionNode {
public:
    IsRfidDetected(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("friendly_supply_zone_exchange", false, "Friendly supply zone exchange"),
            BT::InputPort<bool>("friendly_supply_zone_non_exchange", false, "Friendly supply zone non-exchange"),
            BT::InputPort<bool>("center_gain_point", false, "Center gain point"),
            BT::InputPort<bool>("friendly_fortress_gain_point", false, "Friendly fortress gain point"),
            BT::InputPort<SentryRefereeReceiver*>("key_port")
        };
    }

    BT::NodeStatus tick() override
    {
        return BT::NodeStatus::FAILURE;
    }
};

class IsTargetVisible : public BT::ConditionNode {
public:
    IsTargetVisible(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_topic", "/armor_solver/target", "Target topic"),
            BT::InputPort<double>("timeout", 1.0, "Seconds to consider target valid")
        };
    }

    BT::NodeStatus tick() override
    {
        // This node consumes a string-encoded visibility flag topic rather than rm_interfaces::msg::Target.
        const std::string topic = getInput<std::string>("target_topic").value();
        const double timeout = getInput<double>("timeout").value();

        if (!node_) {
            node_ = make_isolated_node("bt_target_flag_cache");
        }
        if (topic_ != topic || !sub_) {
            topic_ = topic;
            has_msg_ = false;
            last_flag_ = false;
            // 这里读取的是字符串话题，约定解析后大于 0.5 视为“有目标”。
            sub_ = node_->create_subscription<std_msgs::msg::String>(
                topic, rclcpp::SensorDataQoS(),
                [](const std_msgs::msg::String::SharedPtr msg) {
                    const char* raw = msg->data.c_str();
                    char* end = nullptr;
                    const double flag = std::strtod(raw, &end);
                    last_flag_ = (end != raw) && (flag > 0.5);
                    last_time_ = node_->now();
                    has_msg_ = true;
                });
        }

        if (!has_msg_) {
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "IsTargetVisible: no target received yet");
            return BT::NodeStatus::FAILURE;
        }

        const double age = (node_->now() - last_time_).seconds();
        if (age > timeout) {
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "IsTargetVisible: target expired (age=%.2f timeout=%.2f)", age, timeout);
            return BT::NodeStatus::FAILURE;
        }
        if (!last_flag_) {
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "IsTargetVisible: target flag is false");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    static void resetCache()
    {
        sub_.reset();
        node_.reset();
        topic_.clear();
        has_msg_ = false;
        last_flag_ = false;
        last_time_ = rclcpp::Time(0, 0, rclcpp::Clock().get_clock_type());
    }

private:
    inline static rclcpp::Node::SharedPtr node_ = nullptr;
    inline static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ = nullptr;
    inline static std::string topic_;
    inline static bool has_msg_ = false;
    inline static bool last_flag_ = false;
    inline static rclcpp::Time last_time_{};
};

class ChaseTarget : public BT::SyncActionNode {
public:
    ChaseTarget(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!node_) {
            node_ = make_isolated_node("bt_chase_target");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
    }

    static void cleanupSharedResources()
    {
        tf_listener_.reset();
        tf_buffer_.reset();
        node_.reset();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_topic", "/armor_solver/target", "Target topic"),
            BT::InputPort<double>("follow_distance", 2.0, "Follow distance behind target (m)"),
            BT::InputPort<double>("prediction_horizon", 0.2, "Predict target position by velocity * horizon (s)"),
            BT::InputPort<std::string>("topic_name", "goal_pose", "Topic name for PoseStamped goal"),
            BT::InputPort<std::string>("output_frame", "map", "Frame to publish goal in")
        };
    }

    BT::NodeStatus tick() override
    {
        const std::string target_topic = getInput<std::string>("target_topic").value();
        const double follow_distance = getInput<double>("follow_distance").value();
        const double prediction_horizon = getInput<double>("prediction_horizon").value();
        const std::string topic_name = getInput<std::string>("topic_name").value();
        const std::string output_frame = getInput<std::string>("output_frame").value();

        ensureTargetSubscription(target_topic);
        if (!g_has_target || !g_target_node) {
            return BT::NodeStatus::FAILURE;
        }

        const double age = (g_target_node->now() - g_last_target_time).seconds();
        if (age > 1.0 || !g_last_target.tracking) {
            return BT::NodeStatus::FAILURE;
        }

        // Build a follow goal from the latest cached target state.
        const auto& pos = g_last_target.position;
        const auto& vel = g_last_target.velocity;
        const double yaw = g_last_target.yaw;

        // 先根据速度做一小段前向预测，再回退到目标后方形成跟随点。
        const double px = pos.x + vel.x * prediction_horizon;
        const double py = pos.y + vel.y * prediction_horizon;
        const double pz = pos.z + vel.z * prediction_horizon;

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->now();
        goal.header.frame_id = g_last_target.header.frame_id.empty() ? "odom" : g_last_target.header.frame_id;
        goal.pose.position.x = px - follow_distance * std::cos(yaw);
        goal.pose.position.y = py - follow_distance * std::sin(yaw);
        goal.pose.position.z = pz;

        const double face_yaw = std::atan2(py - goal.pose.position.y, px - goal.pose.position.x);
        goal.pose.orientation.z = std::sin(face_yaw * 0.5);
        goal.pose.orientation.w = std::cos(face_yaw * 0.5);

        geometry_msgs::msg::PoseStamped goal_out;
        try {
            goal_out = tf_buffer_->transform(goal, output_frame, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                node_->get_logger(), "ChaseTarget TF %s -> %s failed: %s",
                goal.header.frame_id.c_str(), output_frame.c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }

        // Reuse one publisher per output topic across BT ticks.
        static std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubs;
        if (!pubs.count(topic_name)) {
            pubs[topic_name] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(10));
        }
        pubs[topic_name]->publish(goal_out);
        RCLCPP_INFO(
            node_->get_logger(), "ChaseTarget: publish goal %.2f, %.2f (frame=%s)",
            goal_out.pose.position.x, goal_out.pose.position.y, goal_out.header.frame_id.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class PublishSpinSpeed : public BT::SyncActionNode {
public:
    PublishSpinSpeed(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!node_) {
            node_ = make_isolated_node("bt_publish_spin_speed");
        }
    }

    static void cleanupSharedResources()
    {
        node_.reset();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity (rad/s)"),
            BT::InputPort<double>("duration", 0.0, "Publish then sleep duration in milliseconds"),
            BT::InputPort<std::string>("topic_name", "cmd_spin", "Topic name")
        };
    }

    BT::NodeStatus tick() override
    {
        const double spin_speed = getInput<double>("spin_speed").value();
        const double duration_ms = getInput<double>("duration").value();
        const std::string topic_name = getInput<std::string>("topic_name").value();

        static std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers;
        if (!publishers.count(topic_name)) {
            publishers[topic_name] = node_->create_publisher<std_msgs::msg::Float64>(topic_name, rclcpp::QoS(10));
        }

        std_msgs::msg::Float64 msg;
        msg.data = spin_speed;
        publishers[topic_name]->publish(msg);

        if (duration_ms > 0.0) {
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(duration_ms)));
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;
};

class PublishTwist : public BT::SyncActionNode {
public:
    PublishTwist(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!node_) {
            node_ = make_isolated_node("bt_publish_twist");
        }
    }

    static void cleanupSharedResources()
    {
        node_.reset();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("v_y", 0.0, "Linear Y velocity (m/s)"),
            BT::InputPort<double>("v_yaw", 0.0, "Angular Z velocity (rad/s)"),
            BT::InputPort<double>("v_x", 0.0, "Linear X velocity (m/s)"),
            BT::InputPort<double>("duration", 0.0, "Publish then sleep duration in milliseconds"),
            BT::InputPort<std::string>("topic_name", "cmd_vel", "Topic name")
        };
    }

    BT::NodeStatus tick() override
    {
        const double v_y = getInput<double>("v_y").value();
        const double v_yaw = getInput<double>("v_yaw").value();
        const double v_x = getInput<double>("v_x").value();
        const double duration_ms = getInput<double>("duration").value();
        const std::string topic_name = getInput<std::string>("topic_name").value();

        auto pub = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, rclcpp::QoS(10));
        geometry_msgs::msg::Twist msg;
        msg.linear.x = v_x;
        msg.linear.y = v_y;
        msg.angular.z = v_yaw;
        pub->publish(msg);

        if (duration_ms > 0.0) {
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(duration_ms)));
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;
};

class PubNav2Goal : public BT::SyncActionNode {
public:
    PubNav2Goal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!node_) {
            node_ = make_isolated_node("bt_pub_nav2_goal");
        }
    }

    static void cleanupSharedResources()
    {
        node_.reset();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal", "0;0;0", "Goal pose format: x;y;yaw"),
            BT::InputPort<std::string>("topic_name", "goal_pose", "Topic name"),
            BT::InputPort<double>("republish_interval", 0.0, "Republish interval in seconds (<=0 disables periodic republish)")
        };
    }

    BT::NodeStatus tick() override
    {
        const std::string goal_str = getInput<std::string>("goal").value();
        const std::string topic_name = getInput<std::string>("topic_name").value();
        const double republish_interval = getInput<double>("republish_interval").value();

        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        if (!parseGoal(goal_str, x, y, yaw)) {
            RCLCPP_ERROR(node_->get_logger(), "Invalid goal format, expected x;y;yaw, got: %s", goal_str.c_str());
            return BT::NodeStatus::FAILURE;
        }

        static std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubs;
        if (!pubs.count(topic_name)) {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            pubs[topic_name] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, qos);
        }

        struct TopicGoalState {
            double x = 0.0;
            double y = 0.0;
            double yaw = 0.0;
            bool initialized = false;
            rclcpp::Time last_pub_time{};
        };
        static std::map<std::string, TopicGoalState> state_by_topic;
        auto& state = state_by_topic[topic_name];

        constexpr double kEps = 1e-6;
        const bool same_goal = state.initialized &&
                               std::fabs(state.x - x) < kEps &&
                               std::fabs(state.y - y) < kEps &&
                               std::fabs(state.yaw - yaw) < kEps;
        const auto now = node_->now();
        const bool interval_elapsed =
            !state.initialized ||
            (republish_interval > 0.0 &&
             (now - state.last_pub_time).seconds() >= republish_interval);
        if (same_goal && !interval_elapsed) {
            return BT::NodeStatus::SUCCESS;
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.z = std::sin(yaw * 0.5);
        pose_msg.pose.orientation.w = std::cos(yaw * 0.5);

        pubs[topic_name]->publish(pose_msg);
        state.x = x;
        state.y = y;
        state.yaw = yaw;
        state.initialized = true;
        state.last_pub_time = now;
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool parseGoal(const std::string& input, double& x, double& y, double& yaw) const
    {
        std::stringstream ss(input);
        std::string item;
        std::vector<std::string> tokens;
        while (std::getline(ss, item, ';')) {
            tokens.push_back(item);
        }
        if (tokens.size() != 3) {
            return false;
        }
        try {
            x = std::stod(tokens[0]);
            y = std::stod(tokens[1]);
            yaw = std::stod(tokens[2]);
        } catch (const std::exception&) {
            return false;
        }
        return true;
    }

    static rclcpp::Node::SharedPtr node_;
};

class PublishCurrentPoseGoal : public BT::SyncActionNode {
public:
    PublishCurrentPoseGoal(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        if (!node_) {
            node_ = make_isolated_node("bt_publish_current_pose_goal");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name", "goal_pose", "Topic name"),
            BT::InputPort<std::string>("map_frame", "map", "Map frame"),
            BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame")
        };
    }

    static void cleanupSharedResources()
    {
        tf_listener_.reset();
        tf_buffer_.reset();
        node_.reset();
    }

    BT::NodeStatus tick() override
    {
        const std::string topic_name = getInput<std::string>("topic_name").value();
        const std::string map_frame = getInput<std::string>("map_frame").value();
        const std::string robot_frame = getInput<std::string>("robot_frame").value();

        static std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pubs;
        if (!pubs.count(topic_name)) {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            pubs[topic_name] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, qos);
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(map_frame, robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "PublishCurrentPoseGoal: TF %s -> %s failed: %s",
                map_frame.c_str(), robot_frame.c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = node_->now();
        pose_msg.header.frame_id = map_frame;
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;
        pose_msg.pose.orientation = transform.transform.rotation;

        pubs[topic_name]->publish(pose_msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

inline rclcpp::Node::SharedPtr PublishSpinSpeed::node_ = nullptr;
inline rclcpp::Node::SharedPtr PublishTwist::node_ = nullptr;
inline rclcpp::Node::SharedPtr PubNav2Goal::node_ = nullptr;
inline rclcpp::Node::SharedPtr PublishCurrentPoseGoal::node_ = nullptr;
inline std::shared_ptr<tf2_ros::Buffer> PublishCurrentPoseGoal::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> PublishCurrentPoseGoal::tf_listener_ = nullptr;
inline rclcpp::Node::SharedPtr ChaseTarget::node_ = nullptr;
inline std::shared_ptr<tf2_ros::Buffer> ChaseTarget::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> ChaseTarget::tf_listener_ = nullptr;

inline void cleanupPbExtraNodeResources()
{
    // 退出前释放行为树内部创建的辅助节点，避免持有悬挂 ROS 资源。
    g_target_sub.reset();
    g_target_node.reset();
    g_target_topic.clear();
    g_has_target = false;
    ChaseTarget::cleanupSharedResources();
    PublishSpinSpeed::cleanupSharedResources();
    PublishTwist::cleanupSharedResources();
    PubNav2Goal::cleanupSharedResources();
    PublishCurrentPoseGoal::cleanupSharedResources();
    IsTargetVisible::resetCache();
    IsAllyOutpostDestroyed::resetCache();
    IsEcDecisionGoOut::resetCache();
    IsEcDecisionGoHome::resetCache();
    LowHpLatched::reset();
}

#endif  // ICBK_SENTRY_BT_PB_EXTRA_NODES_H
