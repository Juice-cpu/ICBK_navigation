#ifndef PATROL_BETWEEN_POINTS_H
#define PATROL_BETWEEN_POINTS_H

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "referee_receiver.h"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <map>
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
    bool ensurePoseAvailable();
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
    bool publishGoal(const geometry_msgs::msg::PoseStamped& pose, bool force_log);
    bool currentRobotPose(geometry_msgs::msg::PoseStamped& pose) const;
    bool currentGoalReached() const;
    static bool parseRoute(const std::string& route_str, std::vector<geometry_msgs::msg::PoseStamped>& route);
    static bool parseGoalToken(const std::string& token, geometry_msgs::msg::PoseStamped& pose);
    static double normalizeAngle(double angle);

    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
    static std::atomic<bool> executor_running_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    static std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;

    mutable std::mutex state_mutex_;
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
    std::string goal_topic_;
    std::string map_frame_;
    std::string robot_frame_;
    int hp_switch_threshold_;
    int hp_abort_threshold_;
    double target_timeout_;
    double position_tolerance_;
    double yaw_tolerance_;
    double republish_interval_;

    std::atomic<bool> has_target_;
    std::atomic<bool> target_active_;
    std::atomic<int64_t> last_target_time_ns_;
    geometry_msgs::msg::PoseStamped current_goal_;
    rclcpp::Time last_goal_publish_time_;
    bool has_current_goal_;
};

inline rclcpp::Node::SharedPtr PatrolBetweenPoints::node_ = nullptr;
inline std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> PatrolBetweenPoints::executor_ = nullptr;
inline std::thread PatrolBetweenPoints::spin_thread_;
inline std::atomic<bool> PatrolBetweenPoints::executor_running_ = false;
inline std::shared_ptr<tf2_ros::Buffer> PatrolBetweenPoints::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> PatrolBetweenPoints::tf_listener_ = nullptr;
inline std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
    PatrolBetweenPoints::publishers_;

#endif  // PATROL_BETWEEN_POINTS_H
