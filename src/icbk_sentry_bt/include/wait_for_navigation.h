#ifndef ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H
#define ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

class WaitForNavigation : public BT::StatefulActionNode {
public:
    WaitForNavigation(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
          navigation_done_(false),
          navigation_result_(BT::NodeStatus::RUNNING) {
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_wait_for_navigation", options);
            navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                node_, "navigate_to_pose");
        }
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("goal", "0;0;0", "Goal pose format: x;y;yaw"),
            BT::InputPort<double>("timeout", 30.0, "Timeout in seconds")
        };
    }

    static rclcpp::Node::SharedPtr shared_node() {
        return node_;
    }

    static void cleanupSharedResources() {
        navigation_client_.reset();
        node_.reset();
    }

    BT::NodeStatus onStart() override {
        if (!ensureActionServer()) {
            return BT::NodeStatus::FAILURE;
        }

        const std::string goal_str = getInput<std::string>("goal").value();
        geometry_msgs::msg::PoseStamped goal_pose;
        if (!parseGoal(goal_str, goal_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "WaitForNavigation: invalid goal format: %s", goal_str.c_str());
            return BT::NodeStatus::FAILURE;
        }

        navigation_done_ = false;
        navigation_result_ = BT::NodeStatus::RUNNING;
        start_time_ = node_->now();
        navigation_goal_handle_.reset();

        cancelExistingGoals();

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "WaitForNavigation: goal rejected");
                    navigation_result_ = BT::NodeStatus::FAILURE;
                    navigation_done_ = true;
                    return;
                }
                navigation_goal_handle_ = handle;
                RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: goal accepted");
            };
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        navigation_result_ = BT::NodeStatus::SUCCESS;
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                    case rclcpp_action::ResultCode::ABORTED:
                    default:
                        navigation_result_ = BT::NodeStatus::FAILURE;
                        break;
                }
                navigation_done_ = true;
            };

        auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);
        if (goal_handle_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "WaitForNavigation: failed to send goal");
            return BT::NodeStatus::FAILURE;
        }

        navigation_goal_handle_ = goal_handle_future.get();
        if (!navigation_goal_handle_) {
            RCLCPP_ERROR(node_->get_logger(), "WaitForNavigation: goal handle unavailable");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: navigating to %.2f, %.2f",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const double timeout = getInput<double>("timeout").value();
        const double elapsed = (node_->now() - start_time_).seconds();
        if (elapsed > timeout) {
            RCLCPP_WARN(node_->get_logger(), "WaitForNavigation: timeout after %.1f seconds", elapsed);
            cancelGoal();
            navigation_done_ = true;
            navigation_result_ = BT::NodeStatus::FAILURE;
            return BT::NodeStatus::FAILURE;
        }

        if (navigation_done_) {
            return navigation_result_;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        cancelGoal();
        navigation_done_ = true;
        navigation_result_ = BT::NodeStatus::FAILURE;
        RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: Halted");
    }

private:
    bool ensureActionServer() {
        if (!navigation_client_) {
            return false;
        }
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "WaitForNavigation: waiting for NavigateToPose action server");
            return false;
        }
        return true;
    }

    bool cancelExistingGoals() {
        if (!navigation_client_) {
            return false;
        }

        auto cancel_future = navigation_client_->async_cancel_all_goals();
        const auto status = cancel_future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready) {
            RCLCPP_WARN(node_->get_logger(), "WaitForNavigation: timeout while canceling existing goals");
            return false;
        }

        navigation_goal_handle_.reset();
        RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: requested cancellation of existing navigation goals");
        return true;
    }

    static bool parseGoal(const std::string& input, geometry_msgs::msg::PoseStamped& goal_pose) {
        std::stringstream ss(input);
        std::string item;
        std::vector<std::string> tokens;
        while (std::getline(ss, item, ';')) {
            tokens.push_back(item);
        }
        if (tokens.size() != 3) {
            return false;
        }

        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        try {
            x = std::stod(tokens[0]);
            y = std::stod(tokens[1]);
            yaw = std::stod(tokens[2]);
        } catch (const std::exception&) {
            return false;
        }

        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.z = std::sin(yaw * 0.5);
        goal_pose.pose.orientation.w = std::cos(yaw * 0.5);
        return true;
    }

    void cancelGoal() {
        if (navigation_client_ && navigation_goal_handle_) {
            navigation_client_->async_cancel_goal(navigation_goal_handle_);
            navigation_goal_handle_.reset();
        }
    }

    static rclcpp::Node::SharedPtr node_;
    static rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    bool navigation_done_;
    BT::NodeStatus navigation_result_;
    rclcpp::Time start_time_;
};

inline rclcpp::Node::SharedPtr WaitForNavigation::node_ = nullptr;
inline rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr WaitForNavigation::navigation_client_ = nullptr;

#endif // ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H
