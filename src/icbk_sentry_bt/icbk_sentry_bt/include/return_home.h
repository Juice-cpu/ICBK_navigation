#ifndef return_home_h
#define return_home_h

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <atomic>
#include <mutex>

class ReturnHome : public BT::StatefulActionNode {
public:
    ReturnHome(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    static rclcpp::Node::SharedPtr shared_node();
    static void cleanupSharedResources();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool ensureActionServer();
    bool cancelExistingGoals();
    void cancelCurrentGoal();
    static bool sameHomePoint(const geometry_msgs::msg::PoseStamped& a,
                              const geometry_msgs::msg::PoseStamped& b);

    static rclcpp::Node::SharedPtr node_;
    static rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
    static std::atomic<uint64_t> goal_sequence_;
    static std::mutex shared_state_mutex_;
    static rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr shared_goal_handle_;
    static std::atomic<bool> shared_goal_active_;
    static std::atomic<bool> shared_navigation_done_;
    static std::atomic<BT::NodeStatus> shared_navigation_result_;
    static geometry_msgs::msg::PoseStamped shared_home_point_;
    static std::atomic<uint64_t> shared_goal_id_;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    bool navigation_done_;
    BT::NodeStatus navigation_result_;
    uint64_t active_goal_id_;
};

inline rclcpp::Node::SharedPtr ReturnHome::node_ = nullptr;
inline rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr ReturnHome::navigation_client_ = nullptr;
inline std::atomic<uint64_t> ReturnHome::goal_sequence_{0};
inline std::mutex ReturnHome::shared_state_mutex_;
inline rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr ReturnHome::shared_goal_handle_ = nullptr;
inline std::atomic<bool> ReturnHome::shared_goal_active_{false};
inline std::atomic<bool> ReturnHome::shared_navigation_done_{false};
inline std::atomic<BT::NodeStatus> ReturnHome::shared_navigation_result_{BT::NodeStatus::FAILURE};
inline geometry_msgs::msg::PoseStamped ReturnHome::shared_home_point_{};
inline std::atomic<uint64_t> ReturnHome::shared_goal_id_{0};

#endif // return_home_h
