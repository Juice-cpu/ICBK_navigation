#ifndef PATROL_BETWEEN_POINTS_H
#define PATROL_BETWEEN_POINTS_H

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "referee_receiver.h"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class PatrolBetweenPoints : public BT::StatefulActionNode {
public:
    PatrolBetweenPoints(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    static rclcpp::Node::SharedPtr shared_node();
    static void cleanupSharedResources();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool ensureActionServer();
    bool cancelExistingGoals();
    void cancelCurrentGoal(const char* reason);
    BT::NodeStatus sendNextGoal();
    bool loadInputs();
    void startMonitorLoop();
    void stopMonitorLoop();
    void monitorLoop();
    bool shouldYieldForHp() const;
    bool targetCurrentlyVisible() const;
    bool routeShouldBeHigh() const;
    bool refreshRoute(bool force_reset);
    void updateRouteForCurrentHp();
    static bool parseRoute(const std::string& route_str, std::vector<geometry_msgs::msg::PoseStamped>& route);
    static bool parseGoalToken(const std::string& token, geometry_msgs::msg::PoseStamped& pose);

    static rclcpp::Node::SharedPtr node_;
    static rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
    static std::atomic<bool> executor_running_;

    mutable std::mutex state_mutex_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_sub_;

    std::atomic<bool> navigation_done_;
    std::atomic<BT::NodeStatus> navigation_result_;
    std::atomic<bool> patrol_active_;
    std::atomic<bool> stop_monitor_;
    std::atomic<bool> yield_requested_;
    std::atomic<bool> restart_requested_;
    std::atomic<bool> paused_for_target_;
    std::thread monitor_thread_;

    std::vector<geometry_msgs::msg::PoseStamped> active_route_;
    std::size_t route_index_;
    bool using_high_route_;

    SentryRefereeReceiver* ref_ptr_;
    std::string high_points_;
    std::string low_points_;
    std::string target_topic_;
    int hp_switch_threshold_;
    int hp_abort_threshold_;
    double target_timeout_;

    std::atomic<bool> has_target_;
    std::atomic<bool> target_active_;
    std::atomic<int64_t> last_target_time_ns_;
};

inline rclcpp::Node::SharedPtr PatrolBetweenPoints::node_ = nullptr;
inline rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr PatrolBetweenPoints::navigation_client_ = nullptr;
inline std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> PatrolBetweenPoints::executor_ = nullptr;
inline std::thread PatrolBetweenPoints::spin_thread_;
inline std::atomic<bool> PatrolBetweenPoints::executor_running_ = false;

#endif  // PATROL_BETWEEN_POINTS_H
