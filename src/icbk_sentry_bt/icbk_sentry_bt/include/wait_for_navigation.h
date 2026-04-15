#ifndef ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H
#define ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class WaitForNavigation : public BT::StatefulActionNode {
public:
    WaitForNavigation(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
          navigation_done_(false),
          navigation_result_(BT::NodeStatus::RUNNING),
          position_tolerance_(0.25),
          yaw_tolerance_(0.35),
          republish_interval_(0.5)
    {
        if (!node_) {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            node_ = rclcpp::Node::make_shared("bt_wait_for_navigation", options);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_);
            executor_running_ = true;
            spin_thread_ = std::thread([]() {
                if (executor_) {
                    executor_->spin();
                }
            });
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal", "0;0;0", "Goal pose format: x;y;yaw"),
            BT::InputPort<double>("timeout", 30.0, "Timeout in seconds"),
            BT::InputPort<std::string>("topic_name", "goal_pose", "Goal topic name"),
            BT::InputPort<double>("position_tolerance", 0.25, "Goal position tolerance in meters"),
            BT::InputPort<double>("yaw_tolerance", 0.35, "Goal yaw tolerance in radians"),
            BT::InputPort<double>("republish_interval", 0.5, "Goal republish interval in seconds"),
            BT::InputPort<std::string>("map_frame", "map", "Global frame"),
            BT::InputPort<std::string>("robot_frame", "base_link", "Robot base frame")
        };
    }

    static rclcpp::Node::SharedPtr shared_node()
    {
        return node_;
    }

    static void cleanupSharedResources()
    {
        executor_running_ = false;
        if (executor_) {
            executor_->cancel();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        publishers_.clear();
        executor_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        node_.reset();
    }

    BT::NodeStatus onStart() override
    {
        const std::string goal_str = getInput<std::string>("goal").value();
        if (!parseGoal(goal_str, goal_pose_)) {
            RCLCPP_ERROR(node_->get_logger(), "WaitForNavigation: invalid goal format: %s", goal_str.c_str());
            return BT::NodeStatus::FAILURE;
        }

        topic_name_ = getInput<std::string>("topic_name").value();
        map_frame_ = getInput<std::string>("map_frame").value();
        robot_frame_ = getInput<std::string>("robot_frame").value();
        position_tolerance_ = getInput<double>("position_tolerance").value();
        yaw_tolerance_ = getInput<double>("yaw_tolerance").value();
        republish_interval_ = getInput<double>("republish_interval").value();

        navigation_done_ = false;
        navigation_result_ = BT::NodeStatus::RUNNING;
        start_time_ = node_->now();

        if (!publishGoal(true)) {
            return BT::NodeStatus::FAILURE;
        }

        if (goalReached()) {
            navigation_done_ = true;
            navigation_result_ = BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: navigating to %.2f, %.2f via topic %s",
                    goal_pose_.pose.position.x, goal_pose_.pose.position.y, topic_name_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (republish_interval_ > 0.0 &&
            (node_->now() - last_publish_time_).seconds() >= republish_interval_) {
            if (!publishGoal(false)) {
                return BT::NodeStatus::FAILURE;
            }
        }

        if (goalReached()) {
            navigation_done_ = true;
            navigation_result_ = BT::NodeStatus::SUCCESS;
            return BT::NodeStatus::SUCCESS;
        }

        const double timeout = getInput<double>("timeout").value();
        const double elapsed = (node_->now() - start_time_).seconds();
        if (elapsed > timeout) {
            RCLCPP_WARN(node_->get_logger(), "WaitForNavigation: timeout after %.1f seconds", elapsed);
            navigation_done_ = true;
            navigation_result_ = BT::NodeStatus::FAILURE;
            return BT::NodeStatus::FAILURE;
        }

        if (navigation_done_) {
            return navigation_result_;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        navigation_done_ = true;
        navigation_result_ = BT::NodeStatus::FAILURE;
        RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: Halted");
    }

private:
    bool publishGoal(bool log_publish)
    {
        if (!publishers_.count(topic_name_)) {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            publishers_[topic_name_] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, qos);
        }

        goal_pose_.header.stamp = node_->now();
        goal_pose_.header.frame_id = map_frame_;
        publishers_[topic_name_]->publish(goal_pose_);
        last_publish_time_ = goal_pose_.header.stamp;
        if (log_publish) {
            RCLCPP_INFO(node_->get_logger(), "WaitForNavigation: published goal on %s", topic_name_.c_str());
        }
        return true;
    }

    static bool parseGoal(const std::string& input, geometry_msgs::msg::PoseStamped& goal_pose)
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

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    bool goalReached()
    {
        if (!tf_buffer_) {
            return false;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "WaitForNavigation: TF %s -> %s failed: %s",
                map_frame_.c_str(), robot_frame_.c_str(), ex.what());
            return false;
        }

        const double dx = goal_pose_.pose.position.x - transform.transform.translation.x;
        const double dy = goal_pose_.pose.position.y - transform.transform.translation.y;
        const double distance = std::hypot(dx, dy);

        tf2::Quaternion goal_q;
        tf2::fromMsg(goal_pose_.pose.orientation, goal_q);
        double goal_roll = 0.0;
        double goal_pitch = 0.0;
        double goal_yaw = 0.0;
        tf2::Matrix3x3(goal_q).getRPY(goal_roll, goal_pitch, goal_yaw);

        tf2::Quaternion robot_q;
        tf2::fromMsg(transform.transform.rotation, robot_q);
        double robot_roll = 0.0;
        double robot_pitch = 0.0;
        double robot_yaw = 0.0;
        tf2::Matrix3x3(robot_q).getRPY(robot_roll, robot_pitch, robot_yaw);

        const double yaw_error = std::fabs(normalizeAngle(goal_yaw - robot_yaw));
        return distance <= position_tolerance_ && yaw_error <= yaw_tolerance_;
    }

    static rclcpp::Node::SharedPtr node_;
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    static std::thread spin_thread_;
    static std::atomic<bool> executor_running_;
    static std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;

    bool navigation_done_;
    BT::NodeStatus navigation_result_;
    rclcpp::Time start_time_;
    rclcpp::Time last_publish_time_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    std::string topic_name_;
    std::string map_frame_;
    std::string robot_frame_;
    double position_tolerance_;
    double yaw_tolerance_;
    double republish_interval_;
};

inline rclcpp::Node::SharedPtr WaitForNavigation::node_ = nullptr;
inline std::shared_ptr<tf2_ros::Buffer> WaitForNavigation::tf_buffer_ = nullptr;
inline std::shared_ptr<tf2_ros::TransformListener> WaitForNavigation::tf_listener_ = nullptr;
inline std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> WaitForNavigation::executor_ = nullptr;
inline std::thread WaitForNavigation::spin_thread_;
inline std::atomic<bool> WaitForNavigation::executor_running_ = false;
inline std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
    WaitForNavigation::publishers_;

#endif // ICBK_SENTRY_BT_WAIT_FOR_NAVIGATION_H
