#ifndef ICBK_SENTRY_BT_CHASE_ENEMY_H
#define ICBK_SENTRY_BT_CHASE_ENEMY_H

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

/**
 * @brief Action node that chases an enemy target using Nav2
 * 
 * This node implements the chase behavior similar to armor_chaser,
 * but as a BehaviorTree node that can be controlled by the behavior tree.
 * 
 * State machine:
 * - SEARCHING: No target or target lost
 * - CHASING: Actively chasing target
 * - TEMP_LOST: Target temporarily lost but within grace period
 * - LOST: Target lost beyond timeout
 */
class ChaseEnemy : public BT::StatefulActionNode {
public:
    ChaseEnemy(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
          state_(State::SEARCHING),
          goal_active_(false),
          filtered_goal_initialized_(false) {
        
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_chase_enemy", options);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");
        }
    }

    static void cleanupSharedResources() {
        action_client_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        node_.reset();
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_topic", "/armor_solver/target", "Target topic to subscribe"),
            BT::InputPort<double>("follow_distance", 2.0, "Distance to stay behind target (m)"),
            BT::InputPort<double>("follow_distance_max", 2.0, "Maximum follow distance (m)"),
            BT::InputPort<double>("prediction_horizon", 0.2, "Time horizon for velocity prediction (s)"),
            BT::InputPort<double>("smoothing_alpha", 0.7, "Low-pass filter coefficient"),
            BT::InputPort<double>("update_distance_threshold", 0.5, "Distance threshold to update goal (m)"),
            BT::InputPort<double>("replan_period", 0.3, "Force replan period (s)"),
            BT::InputPort<double>("lost_timeout", 1.0, "Timeout before considering target lost (s)"),
            BT::InputPort<double>("lost_grace", 0.4, "Grace period for temporary loss (s)"),
            BT::InputPort<std::string>("goal_frame", "map", "Frame for navigation goals")
        };
    }

    BT::NodeStatus onStart() override {
        const std::string target_topic = getInput<std::string>("target_topic").value();
        
        // Subscribe to target topic
        if (!target_sub_ || target_topic_ != target_topic) {
            target_topic_ = target_topic;
            target_sub_ = node_->create_subscription<rm_interfaces::msg::Target>(
                target_topic, rclcpp::SensorDataQoS(),
                std::bind(&ChaseEnemy::targetCallback, this, std::placeholders::_1));
        }

        // Reset state
        state_ = State::SEARCHING;
        goal_active_ = false;
        filtered_goal_initialized_ = false;
        last_target_time_ = node_->now();
        last_goal_send_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

        RCLCPP_INFO(node_->get_logger(), "ChaseEnemy: Started chasing");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // Wait for action server
        if (!action_client_->wait_for_action_server(0s)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
                                "Waiting for nav2 action server...");
            return BT::NodeStatus::RUNNING;
        }

        // Update state based on time since last target
        const double since_last_msg = (node_->now() - last_target_time_).seconds();
        const double lost_timeout = getInput<double>("lost_timeout").value();
        const double lost_grace = getInput<double>("lost_grace").value();

        if (since_last_msg > lost_timeout) {
            if (state_ != State::LOST) {
                RCLCPP_WARN(node_->get_logger(), "ChaseEnemy: Target lost, stopping chase");
            }
            state_ = State::LOST;
        } else if (state_ == State::TEMP_LOST && since_last_msg > lost_grace) {
            state_ = State::SEARCHING;
        }

        // Handle chasing or temporary loss
        if (state_ == State::CHASING || state_ == State::TEMP_LOST) {
            sendOrUpdateGoal();
            return BT::NodeStatus::RUNNING;
        } else if (state_ == State::LOST) {
            // Cancel goal if active
            if (goal_active_) {
                action_client_->async_cancel_all_goals();
                goal_active_ = false;
            }
            return BT::NodeStatus::FAILURE;
        }

        // SEARCHING state
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Cancel any active goal
        if (goal_active_) {
            action_client_->async_cancel_all_goals();
            goal_active_ = false;
        }
        RCLCPP_INFO(node_->get_logger(), "ChaseEnemy: Halted");
    }

private:
    enum class State { SEARCHING, CHASING, TEMP_LOST, LOST };

    void targetCallback(const rm_interfaces::msg::Target::SharedPtr msg) {
        rm_interfaces::msg::Target target_in_goal = *msg;
        const std::string goal_frame = getInput<std::string>("goal_frame").value();

        // Transform to goal frame if needed
        if (!goal_frame.empty() && msg->header.frame_id != goal_frame) {
            try {
                geometry_msgs::msg::PoseStamped pose_src;
                pose_src.header = msg->header;
                pose_src.pose.position = msg->position;
                tf2::Quaternion q_src;
                q_src.setRPY(0.0, 0.0, msg->yaw);
                pose_src.pose.orientation = tf2::toMsg(q_src);

                geometry_msgs::msg::PoseStamped pose_out;
                tf_buffer_->transform(pose_src, pose_out, goal_frame, tf2::durationFromSec(0.2));
                target_in_goal.position = pose_out.pose.position;
                target_in_goal.yaw = tf2::getYaw(pose_out.pose.orientation);

                geometry_msgs::msg::Vector3Stamped vel_src;
                vel_src.header = msg->header;
                vel_src.vector = msg->velocity;
                geometry_msgs::msg::Vector3Stamped vel_out;
                tf_buffer_->transform(vel_src, vel_out, goal_frame, tf2::durationFromSec(0.2));
                target_in_goal.velocity = vel_out.vector;

                target_in_goal.header.frame_id = goal_frame;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                    "TF transform failed: %s", ex.what());
                return;
            }
        }

        last_target_msg_ = target_in_goal;
        last_target_time_ = node_->now();
        state_ = target_in_goal.tracking ? State::CHASING : State::TEMP_LOST;
    }

    void sendOrUpdateGoal() {
        const double lost_grace = getInput<double>("lost_grace").value();
        
        if (!last_target_msg_.tracking) {
            const double since = (node_->now() - last_target_time_).seconds();
            if (since > lost_grace) {
                return;
            }
        }

        const auto& pos = last_target_msg_.position;
        const auto& vel = last_target_msg_.velocity;
        const double yaw = last_target_msg_.yaw;
        const double prediction_horizon = getInput<double>("prediction_horizon").value();
        const double smoothing_alpha = getInput<double>("smoothing_alpha").value();
        const double follow_distance = getInput<double>("follow_distance").value();
        const double follow_distance_max = getInput<double>("follow_distance_max").value();
        const std::string goal_frame = getInput<std::string>("goal_frame").value();

        // Predict target position
        geometry_msgs::msg::Point predicted = pos;
        predicted.x += vel.x * prediction_horizon;
        predicted.y += vel.y * prediction_horizon;
        predicted.z += vel.z * prediction_horizon;

        // Apply low-pass filter
        if (!filtered_goal_initialized_) {
            filtered_goal_position_ = predicted;
            filtered_goal_initialized_ = true;
        } else {
            filtered_goal_position_.x = smoothing_alpha * predicted.x + 
                                       (1.0 - smoothing_alpha) * filtered_goal_position_.x;
            filtered_goal_position_.y = smoothing_alpha * predicted.y + 
                                       (1.0 - smoothing_alpha) * filtered_goal_position_.y;
            filtered_goal_position_.z = smoothing_alpha * predicted.z + 
                                       (1.0 - smoothing_alpha) * filtered_goal_position_.z;
        }

        // Calculate goal position (behind target along its heading)
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->now();
        goal.header.frame_id = goal_frame;

        goal.pose.position.x = filtered_goal_position_.x - follow_distance * std::cos(yaw);
        goal.pose.position.y = filtered_goal_position_.y - follow_distance * std::sin(yaw);
        goal.pose.position.z = filtered_goal_position_.z;

        // Limit distance
        double rel_dx = goal.pose.position.x - filtered_goal_position_.x;
        double rel_dy = goal.pose.position.y - filtered_goal_position_.y;
        double rel_dist = std::hypot(rel_dx, rel_dy);
        if (rel_dist > follow_distance_max && rel_dist > 1e-6) {
            double scale = follow_distance_max / rel_dist;
            goal.pose.position.x = filtered_goal_position_.x + rel_dx * scale;
            goal.pose.position.y = filtered_goal_position_.y + rel_dy * scale;
        }

        // Face towards target
        const double face_yaw = std::atan2(
            filtered_goal_position_.y - goal.pose.position.y,
            filtered_goal_position_.x - goal.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, face_yaw);
        goal.pose.orientation = tf2::toMsg(q);

        // Check if we need to update goal
        const double update_threshold = getInput<double>("update_distance_threshold").value();
        const double replan_period = getInput<double>("replan_period").value();

        double dx = goal.pose.position.x - last_goal_pose_.pose.position.x;
        double dy = goal.pose.position.y - last_goal_pose_.pose.position.y;
        double dist = std::hypot(dx, dy);

        auto now_time = node_->now();
        double time_since_last_goal = std::numeric_limits<double>::infinity();
        if (last_goal_send_time_.nanoseconds() != 0) {
            time_since_last_goal = (now_time - last_goal_send_time_).seconds();
        }

        if (!goal_active_ || dist > update_threshold || time_since_last_goal > replan_period) {
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose = goal;

            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
                    if (!handle) {
                        RCLCPP_ERROR(node_->get_logger(), "ChaseEnemy: Goal rejected");
                        goal_active_ = false;
                    } else {
                        RCLCPP_INFO(node_->get_logger(), "ChaseEnemy: Goal accepted");
                    }
                };
            send_goal_options.result_callback =
                [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& res) {
                    goal_active_ = false;
                    switch (res.code) {
                        case rclcpp_action::ResultCode::SUCCEEDED:
                            RCLCPP_INFO(node_->get_logger(), "ChaseEnemy: Goal reached");
                            break;
                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_WARN(node_->get_logger(), "ChaseEnemy: Goal aborted");
                            break;
                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_WARN(node_->get_logger(), "ChaseEnemy: Goal canceled");
                            break;
                        default:
                            RCLCPP_ERROR(node_->get_logger(), "ChaseEnemy: Unknown result");
                            break;
                    }
                };

            action_client_->async_send_goal(goal_msg, send_goal_options);
            last_goal_pose_ = goal;
            goal_active_ = true;
            last_goal_send_time_ = now_time;
            RCLCPP_INFO(node_->get_logger(), "ChaseEnemy: Sent goal (%.2f, %.2f)", 
                       goal.pose.position.x, goal.pose.position.y);
        }
    }

    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    static rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

    State state_;
    rm_interfaces::msg::Target last_target_msg_;
    rclcpp::Time last_target_time_;
    geometry_msgs::msg::PoseStamped last_goal_pose_;
    bool goal_active_;
    rclcpp::Time last_goal_send_time_;
    geometry_msgs::msg::Point filtered_goal_position_;
    bool filtered_goal_initialized_;
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    std::string target_topic_;
};

class EnemyVisibleFromTopic : public BT::ConditionNode {
public:
    EnemyVisibleFromTopic(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_enemy_visible_from_topic", options);
        }
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("target_topic", "/armor_solver/target", "Target topic"),
            BT::InputPort<double>("timeout", 1.0, "Seconds to consider target valid")
        };
    }

    static void cleanupSharedResources() {
        target_sub_.reset();
        node_.reset();
        target_topic_.clear();
        has_target_ = false;
    }

    BT::NodeStatus tick() override {
        const std::string topic = getInput<std::string>("target_topic").value();
        const double timeout = getInput<double>("timeout").value();

        if (!target_sub_ || target_topic_ != topic) {
            target_topic_ = topic;
            has_target_ = false;
            target_sub_ = node_->create_subscription<rm_interfaces::msg::Target>(
                topic, rclcpp::SensorDataQoS(),
                [](const rm_interfaces::msg::Target::SharedPtr msg) {
                    last_target_ = *msg;
                    last_target_time_ = node_->now();
                    has_target_ = true;
                });
        }

        if (!has_target_) {
            return BT::NodeStatus::FAILURE;
        }

        const double age = (node_->now() - last_target_time_).seconds();
        if (age > timeout || !last_target_.tracking) {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    static rclcpp::Node::SharedPtr node_;
    static rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    static rm_interfaces::msg::Target last_target_;
    static rclcpp::Time last_target_time_;
    static bool has_target_;
    static std::string target_topic_;
};

// Static member definitions
inline rclcpp::Node::SharedPtr ChaseEnemy::node_ = nullptr;
inline std::shared_ptr<tf2_ros::Buffer> ChaseEnemy::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> ChaseEnemy::tf_listener_ = nullptr;
inline rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr ChaseEnemy::action_client_ = nullptr;
inline rclcpp::Node::SharedPtr EnemyVisibleFromTopic::node_ = nullptr;
inline rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr EnemyVisibleFromTopic::target_sub_ = nullptr;
inline rm_interfaces::msg::Target EnemyVisibleFromTopic::last_target_{};
inline rclcpp::Time EnemyVisibleFromTopic::last_target_time_{};
inline bool EnemyVisibleFromTopic::has_target_ = false;
inline std::string EnemyVisibleFromTopic::target_topic_;

#endif // ICBK_SENTRY_BT_CHASE_ENEMY_H
