#ifndef ICBK_SENTRY_BT_IS_ATTACKED_H
#define ICBK_SENTRY_BT_IS_ATTACKED_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "referee_receiver.h"
#include "pb_extra_nodes.h"  // For global referee source
#include "rclcpp/rclcpp.hpp"
#include <cmath>

/**
 * @brief Condition node that checks if the robot is being attacked
 * 
 * This node checks if the robot's HP has decreased, indicating an attack.
 * It also outputs the approximate direction of the attack based on armor ID.
 * 
 * Note: The current RefereeData structure doesn't include armor_id or 
 * hp_deduction_reason, so this is a simplified version that detects
 * HP decrease and estimates direction.
 */
class IsAttacked : public BT::ConditionNode {
public:
    IsAttacked(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), last_hp_(0), initialized_(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<SentryRefereeReceiver*>("referee_data", "Referee data source pointer"),
            BT::InputPort<int>("hp_drop_min", 1, "Minimum HP drop to consider as attack"),
            BT::OutputPort<double>("gimbal_yaw", "Yaw angle to face the attack direction (rad)"),
            BT::OutputPort<double>("gimbal_pitch", "Pitch angle (usually 0.0)"),
            BT::OutputPort<double>("attack_yaw", "Yaw angle of incoming attack (rad)")
        };
    }

    BT::NodeStatus tick() override {
        SentryRefereeReceiver* ref_ptr = nullptr;
        try {
            auto ref = getInput<SentryRefereeReceiver*>("referee_data");
            if (ref) {
                ref_ptr = *ref;
            }
        } catch (const std::exception&) {
            // Fallback to global pointer
        }

        if (!ref_ptr && g_referee_source) {
            ref_ptr = g_referee_source;
        }

        if (!ref_ptr) {
            throw BT::RuntimeError("missing required input [referee_data]: referee source is not set");
        }

        const auto data = ref_ptr->getLatestData();

        // Initialize on first tick
        if (!initialized_) {
            last_hp_ = data.sentry_hp;
            initialized_ = true;
            return BT::NodeStatus::FAILURE;
        }

        const int hp_drop_min = getInput<int>("hp_drop_min").value();
        const int hp_loss = static_cast<int>(last_hp_) - static_cast<int>(data.sentry_hp);
        const bool is_attacked = hp_loss >= hp_drop_min;

        if (is_attacked) {
            RCLCPP_INFO(rclcpp::get_logger("IsAttacked"), 
                       "Attack detected! HP decreased from %d to %d (loss=%d)", 
                       last_hp_, data.sentry_hp, hp_loss);

            // Estimate attack direction based on robot's current angle
            // Since we don't have armor_id, we'll use a simple heuristic:
            // Assume attack comes from the direction opposite to robot's facing direction
            // This is a simplified approach - in real implementation, you'd need armor_id
            float estimated_attack_yaw = data.angle + M_PI; // Opposite direction
            if (estimated_attack_yaw > M_PI) {
                estimated_attack_yaw -= 2 * M_PI;
            } else if (estimated_attack_yaw < -M_PI) {
                estimated_attack_yaw += 2 * M_PI;
            }

            setOutput("gimbal_yaw", static_cast<double>(estimated_attack_yaw));
            setOutput("gimbal_pitch", 0.0);
            setOutput("attack_yaw", static_cast<double>(estimated_attack_yaw));

            last_hp_ = data.sentry_hp;
            return BT::NodeStatus::SUCCESS;
        }

        // Update last HP even if not attacked
        last_hp_ = data.sentry_hp;
        return BT::NodeStatus::FAILURE;
    }

private:
    uint16_t last_hp_;
    bool initialized_;
};

#endif // ICBK_SENTRY_BT_IS_ATTACKED_H
