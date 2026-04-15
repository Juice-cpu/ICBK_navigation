#ifndef ICBK_SENTRY_BT_IS_DETECT_ENEMY_H
#define ICBK_SENTRY_BT_IS_DETECT_ENEMY_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "pb_extra_nodes.h"  // For global target subscription variables
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/target.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>

/**
 * @brief Condition node that checks if an enemy is detected
 * 
 * This node subscribes to a target topic and checks if:
 * 1. A target is visible and tracking
 * 2. The target ID matches expected armor IDs
 * 3. The target is within maximum distance
 */
class IsDetectEnemy : public BT::ConditionNode {
public:
    IsDetectEnemy(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_is_detect_enemy", options);
        }
    }

    static void cleanupSharedResources() {
        node_.reset();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_topic", "/armor_solver/target", "Target topic to subscribe"),
            BT::InputPort<std::string>("armor_id", "1;2;3;4;5;7", "Expected armor IDs separated by ';'"),
            BT::InputPort<double>("max_distance", 8.0, "Maximum distance to consider enemy detected"),
            BT::InputPort<double>("timeout", 1.0, "Seconds to consider target valid")
        };
    }

    BT::NodeStatus tick() override {
        const std::string target_topic = getInput<std::string>("target_topic").value();
        const std::string armor_id_str = getInput<std::string>("armor_id").value();
        const double max_distance = getInput<double>("max_distance").value();
        const double timeout = getInput<double>("timeout").value();

        // Parse armor IDs
        std::vector<int> expected_armor_ids;
        std::stringstream ss(armor_id_str);
        std::string item;
        while (std::getline(ss, item, ';')) {
            try {
                expected_armor_ids.push_back(std::stoi(item));
            } catch (const std::exception&) {
                // Ignore invalid IDs
            }
        }

        // Ensure subscription
        ensureTargetSubscription(target_topic);

        if (!g_has_target) {
            RCLCPP_DEBUG(node_->get_logger(), "No target received yet");
            return BT::NodeStatus::FAILURE;
        }

        // Check if target is still valid
        // Use g_target_node->now() to ensure same clock source as g_last_target_time
        if (!g_target_node) {
            RCLCPP_DEBUG(node_->get_logger(), "Target node not initialized");
            return BT::NodeStatus::FAILURE;
        }
        const double age = (g_target_node->now() - g_last_target_time).seconds();
        if (age > timeout) {
            RCLCPP_DEBUG(node_->get_logger(), "Target expired: age=%.2f timeout=%.2f", age, timeout);
            return BT::NodeStatus::FAILURE;
        }

        if (!g_last_target.tracking) {
            RCLCPP_DEBUG(node_->get_logger(), "Target not tracking");
            return BT::NodeStatus::FAILURE;
        }

        // Check distance
        const double distance = std::hypot(
            g_last_target.position.x,
            g_last_target.position.y,
            g_last_target.position.z
        );

        if (distance > max_distance) {
            RCLCPP_DEBUG(node_->get_logger(), "Target too far: %.2f > %.2f", distance, max_distance);
            return BT::NodeStatus::FAILURE;
        }

        // Check armor ID if available
        if (!g_last_target.id.empty()) {
            try {
                int armor_id = std::stoi(g_last_target.id);
                bool is_match = std::find(expected_armor_ids.begin(), 
                                          expected_armor_ids.end(), 
                                          armor_id) != expected_armor_ids.end();
                if (!is_match && !expected_armor_ids.empty()) {
                    RCLCPP_DEBUG(node_->get_logger(), "Armor ID %d not in expected list", armor_id);
                    return BT::NodeStatus::FAILURE;
                }
            } catch (const std::exception&) {
                // If ID parsing fails, continue (accept any target)
            }
        }

        RCLCPP_INFO(node_->get_logger(), 
                    "Enemy detected: distance=%.2f, id=%s, tracking=%s",
                    distance, g_last_target.id.c_str(), 
                    g_last_target.tracking ? "true" : "false");

        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;

    // Reuse the global target subscription from pb_extra_nodes.h
    void ensureTargetSubscription(const std::string& topic) {
        // Use the function from pb_extra_nodes.h
        ::ensureTargetSubscription(topic);
    }
};

inline rclcpp::Node::SharedPtr IsDetectEnemy::node_ = nullptr;

#endif // ICBK_SENTRY_BT_IS_DETECT_ENEMY_H
