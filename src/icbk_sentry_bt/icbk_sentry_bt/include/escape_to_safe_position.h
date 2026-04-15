#ifndef ICBK_SENTRY_BT_ESCAPE_TO_SAFE_POSITION_H
#define ICBK_SENTRY_BT_ESCAPE_TO_SAFE_POSITION_H

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

/**
 * @brief Action node that escapes to a safe position when attacked
 * 
 * This node intelligently selects a safe position based on:
 * 1. Attack direction (avoid moving towards the attack)
 * 2. Costmap (avoid obstacles and high-cost areas)
 * 3. Distance from current position (not too far, not too close)
 * 4. Distance from attack direction (maximize distance from threat)
 */
class EscapeToSafePosition : public BT::StatefulActionNode {
public:
    EscapeToSafePosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
          goal_active_(false),
          costmap_received_(false) {
        
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_escape_to_safe", options);
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
            BT::InputPort<double>("attack_yaw", "Yaw angle of attack direction (rad)"),
            BT::InputPort<std::string>("costmap_topic", "/global_costmap/costmap", "Costmap topic"),
            BT::InputPort<std::string>("robot_frame", "base_link", "Robot base frame"),
            BT::InputPort<std::string>("map_frame", "map", "Map frame"),
            BT::InputPort<double>("escape_distance", 2.0, "Preferred escape distance (m)"),
            BT::InputPort<double>("min_escape_distance", 1.0, "Minimum escape distance (m)"),
            BT::InputPort<double>("max_escape_distance", 4.0, "Maximum escape distance (m)"),
            BT::InputPort<double>("search_radius", 3.0, "Search radius for safe positions (m)"),
            BT::InputPort<int>("num_candidates", 36, "Number of candidate positions to evaluate"),
            BT::InputPort<int>("cost_threshold", 50, "Maximum acceptable cost (0-100)"),
            BT::InputPort<double>("prefer_opposite_direction", 0.7, "Weight for preferring opposite direction (0-1)")
        };
    }

    BT::NodeStatus onStart() override {
        const std::string costmap_topic = getInput<std::string>("costmap_topic").value();
        
        // Subscribe to costmap
        if (!costmap_sub_ || costmap_topic_ != costmap_topic) {
            costmap_topic_ = costmap_topic;
            costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
                costmap_topic, rclcpp::QoS(10).transient_local().reliable(),
                std::bind(&EscapeToSafePosition::costmapCallback, this, std::placeholders::_1));
        }

        goal_active_ = false;
        costmap_received_ = false;

        RCLCPP_INFO(node_->get_logger(), "EscapeToSafePosition: Started");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // Wait for action server
        if (!action_client_->wait_for_action_server(0s)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                "Waiting for nav2 action server...");
            return BT::NodeStatus::RUNNING;
        }

        // Wait for costmap
        if (!costmap_received_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Waiting for costmap...");
            return BT::NodeStatus::RUNNING;
        }

        // If goal is active, just monitor
        if (goal_active_) {
            return BT::NodeStatus::RUNNING;
        }

        // Calculate and send safe position
        geometry_msgs::msg::PoseStamped safe_pose = calculateSafePosition();
        if (safe_pose.header.frame_id.empty()) {
            RCLCPP_WARN(node_->get_logger(), "EscapeToSafePosition: Could not find safe position");
            return BT::NodeStatus::FAILURE;
        }

        sendGoal(safe_pose);
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        if (goal_active_) {
            action_client_->async_cancel_all_goals();
            goal_active_ = false;
        }
        RCLCPP_INFO(node_->get_logger(), "EscapeToSafePosition: Halted");
    }

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = *msg;
        costmap_received_ = true;
    }

    geometry_msgs::msg::PoseStamped calculateSafePosition() {
        const double attack_yaw = getInput<double>("attack_yaw").value();
        const std::string robot_frame = getInput<std::string>("robot_frame").value();
        const std::string map_frame = getInput<std::string>("map_frame").value();
        const double escape_distance = getInput<double>("escape_distance").value();
        const double min_distance = getInput<double>("min_escape_distance").value();
        const double max_distance = getInput<double>("max_escape_distance").value();
        const double search_radius = getInput<double>("search_radius").value();
        const int num_candidates = getInput<int>("num_candidates").value();
        const int cost_threshold = getInput<int>("cost_threshold").value();
        const double prefer_opposite = getInput<double>("prefer_opposite_direction").value();

        // Get robot position in map frame
        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = robot_frame;
        robot_pose.header.stamp = node_->now();
        robot_pose.pose.orientation.w = 1.0;

        try {
            robot_pose = tf_buffer_->transform(robot_pose, map_frame, tf2::durationFromSec(0.5));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", ex.what());
            return geometry_msgs::msg::PoseStamped();
        }

        const double robot_x = robot_pose.pose.position.x;
        const double robot_y = robot_pose.pose.position.y;
        const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

        // Calculate opposite direction to attack (preferred escape direction)
        const double opposite_yaw = attack_yaw + M_PI;
        const double normalized_opposite = std::atan2(std::sin(opposite_yaw), std::cos(opposite_yaw));

        // Generate candidate positions
        std::vector<CandidatePosition> candidates;
        for (int i = 0; i < num_candidates; ++i) {
            const double angle = 2.0 * M_PI * i / num_candidates;
            const double distance = escape_distance + 
                                   (max_distance - min_distance) * 0.3 * 
                                   (std::sin(angle * 2.0) + 1.0) / 2.0; // Vary distance

            const double candidate_x = robot_x + distance * std::cos(angle);
            const double candidate_y = robot_y + distance * std::sin(angle);

            // Check if within search radius
            const double dist_from_robot = std::hypot(candidate_x - robot_x, candidate_y - robot_y);
            if (dist_from_robot < min_distance || dist_from_robot > max_distance) {
                continue;
            }

            // Get cost at this position
            int cost = getCostAtPosition(candidate_x, candidate_y);
            if (cost > cost_threshold) {
                continue; // Skip high-cost positions
            }

            // Calculate score
            // 1. Lower cost is better
            // 2. Prefer positions opposite to attack direction
            // 3. Prefer positions at preferred distance
            const double angle_diff = std::abs(std::atan2(std::sin(angle - normalized_opposite),
                                                          std::cos(angle - normalized_opposite)));
            const double direction_score = 1.0 - (angle_diff / M_PI); // 1.0 if opposite, 0.0 if same
            const double distance_score = 1.0 - std::abs(dist_from_robot - escape_distance) / max_distance;
            const double cost_score = 1.0 - (static_cast<double>(cost) / 100.0);

            const double total_score = 
                cost_score * 0.4 +  // Cost is important
                direction_score * prefer_opposite * 0.4 +  // Prefer opposite direction
                distance_score * 0.2;  // Prefer preferred distance

            candidates.push_back({candidate_x, candidate_y, angle, cost, total_score});
        }

        if (candidates.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No valid candidate positions found");
            return geometry_msgs::msg::PoseStamped();
        }

        // Sort by score (highest first)
        std::sort(candidates.begin(), candidates.end(),
                  [](const CandidatePosition& a, const CandidatePosition& b) {
                      return a.score > b.score;
                  });

        // Select best candidate
        const auto& best = candidates[0];
        RCLCPP_INFO(node_->get_logger(),
                   "Selected safe position: (%.2f, %.2f), cost=%d, score=%.3f",
                   best.x, best.y, best.cost, best.score);

        // Create pose
        geometry_msgs::msg::PoseStamped safe_pose;
        safe_pose.header.frame_id = map_frame;
        safe_pose.header.stamp = node_->now();
        safe_pose.pose.position.x = best.x;
        safe_pose.pose.position.y = best.y;
        safe_pose.pose.position.z = 0.0;

        // Face away from attack direction
        const double face_yaw = best.angle;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, face_yaw);
        safe_pose.pose.orientation = tf2::toMsg(q);

        return safe_pose;
    }

    int getCostAtPosition(double x, double y) {
        if (!costmap_received_ || costmap_.data.empty()) {
            return 100; // Unknown = high cost
        }

        // Convert world coordinates to map coordinates
        const int map_x = static_cast<int>((x - costmap_.info.origin.position.x) / costmap_.info.resolution);
        const int map_y = static_cast<int>((y - costmap_.info.origin.position.y) / costmap_.info.resolution);

        // Check bounds
        if (map_x < 0 || map_x >= static_cast<int>(costmap_.info.width) ||
            map_y < 0 || map_y >= static_cast<int>(costmap_.info.height)) {
            return 100; // Out of bounds = high cost
        }

        const int index = map_y * costmap_.info.width + map_x;
        return static_cast<int>(costmap_.data[index]);
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& pose) {
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = pose;

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
                if (!handle) {
                    RCLCPP_ERROR(node_->get_logger(), "EscapeToSafePosition: Goal rejected");
                    goal_active_ = false;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "EscapeToSafePosition: Goal accepted");
                    goal_active_ = true;
                }
            };
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& res) {
                goal_active_ = false;
                switch (res.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "EscapeToSafePosition: Reached safe position");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(node_->get_logger(), "EscapeToSafePosition: Goal aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(node_->get_logger(), "EscapeToSafePosition: Goal canceled");
                        break;
                    default:
                        RCLCPP_ERROR(node_->get_logger(), "EscapeToSafePosition: Unknown result");
                        break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "EscapeToSafePosition: Sent goal to (%.2f, %.2f)",
                   pose.pose.position.x, pose.pose.position.y);
    }

    struct CandidatePosition {
        double x, y;
        double angle;
        int cost;
        double score;
    };

    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    static rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

    bool goal_active_;
    bool costmap_received_;
    nav_msgs::msg::OccupancyGrid costmap_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    std::string costmap_topic_;
};

// Static member definitions
inline rclcpp::Node::SharedPtr EscapeToSafePosition::node_ = nullptr;
inline std::shared_ptr<tf2_ros::Buffer> EscapeToSafePosition::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> EscapeToSafePosition::tf_listener_ = nullptr;
inline rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr EscapeToSafePosition::action_client_ = nullptr;

#endif // ICBK_SENTRY_BT_ESCAPE_TO_SAFE_POSITION_H
