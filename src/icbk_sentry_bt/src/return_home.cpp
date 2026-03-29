#include "return_home.h"

#include <chrono>
#include <cmath>

ReturnHome::ReturnHome(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      navigation_done_(false),
      navigation_result_(BT::NodeStatus::RUNNING),
      active_goal_id_(0) {
    if (!node_) {
        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        node_ = rclcpp::Node::make_shared("return_home_node", options);
        navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }
}

BT::PortsList ReturnHome::providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("home_point") };
}

rclcpp::Node::SharedPtr ReturnHome::shared_node() {
    return node_;
}

void ReturnHome::cleanupSharedResources() {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    shared_goal_handle_.reset();
    shared_goal_active_ = false;
    shared_navigation_done_ = false;
    shared_navigation_result_ = BT::NodeStatus::FAILURE;
    shared_goal_id_ = 0;
    navigation_client_.reset();
    node_.reset();
}

bool ReturnHome::ensureActionServer() {
    if (!navigation_client_) {
        return false;
    }
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "ReturnHome: waiting for NavigateToPose action server");
        return false;
    }
    return true;
}

bool ReturnHome::cancelExistingGoals() {
    if (!navigation_client_) {
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "ReturnHome: cancelExistingGoals");
    auto cancel_future = navigation_client_->async_cancel_all_goals();
    const auto status = cancel_future.wait_for(std::chrono::seconds(1));
    if (status != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(), "ReturnHome: timeout while canceling existing goals");
        return false;
    }

    navigation_goal_handle_.reset();
    RCLCPP_INFO(node_->get_logger(), "ReturnHome: requested cancellation of existing navigation goals");
    return true;
}

void ReturnHome::cancelCurrentGoal() {
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(shared_state_mutex_);
        if (!navigation_client_ || !shared_goal_handle_) {
            return;
        }
        goal_handle = shared_goal_handle_;
        shared_goal_handle_.reset();
        shared_goal_active_ = false;
        shared_navigation_done_ = true;
        shared_navigation_result_ = BT::NodeStatus::FAILURE;
    }
    navigation_goal_handle_.reset();
    try {
        navigation_client_->async_cancel_goal(goal_handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "ReturnHome[%llu]: cancel ignored for unknown goal handle: %s",
                    static_cast<unsigned long long>(active_goal_id_),
                    ex.what());
    }
}

BT::NodeStatus ReturnHome::onStart() {
    if (!ensureActionServer()) {
        return BT::NodeStatus::FAILURE;
    }

    auto home_point = getInput<geometry_msgs::msg::PoseStamped>("home_point");
    if (!home_point) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [home_point]");
        return BT::NodeStatus::FAILURE;
    }

    const auto requested_home = home_point.value();
    {
        std::lock_guard<std::mutex> lock(shared_state_mutex_);
        if (shared_goal_active_ && !shared_navigation_done_ && sameHomePoint(shared_home_point_, requested_home)) {
            if (shared_goal_handle_) {
                active_goal_id_ = shared_goal_id_.load();
                navigation_goal_handle_ = shared_goal_handle_;
                navigation_done_ = false;
                navigation_result_ = BT::NodeStatus::RUNNING;
                RCLCPP_INFO(node_->get_logger(),
                            "ReturnHome[%llu]: resume existing home goal",
                            static_cast<unsigned long long>(active_goal_id_));
                return BT::NodeStatus::RUNNING;
            }
            // Handle lost: re-issue goal.
            shared_goal_active_ = false;
            shared_navigation_done_ = true;
            shared_navigation_result_ = BT::NodeStatus::FAILURE;
            shared_goal_id_ = 0;
        }
    }

    cancelExistingGoals();

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = requested_home;
    active_goal_id_ = ++goal_sequence_;

    navigation_done_ = false;
    navigation_result_ = BT::NodeStatus::RUNNING;
    navigation_goal_handle_.reset();
    {
        std::lock_guard<std::mutex> lock(shared_state_mutex_);
        shared_goal_handle_.reset();
        shared_goal_active_ = true;
        shared_navigation_done_ = false;
        shared_navigation_result_ = BT::NodeStatus::RUNNING;
        shared_home_point_ = requested_home;
        shared_goal_id_ = active_goal_id_;
    }

    RCLCPP_INFO(node_->get_logger(),
                "ReturnHome[%llu]: start home=(%.2f, %.2f)",
                static_cast<unsigned long long>(active_goal_id_),
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
            if (!handle) {
                RCLCPP_ERROR(node_->get_logger(),
                             "ReturnHome[%llu]: navigation goal rejected",
                             static_cast<unsigned long long>(active_goal_id_));
                navigation_result_ = BT::NodeStatus::FAILURE;
                navigation_done_ = true;
                std::lock_guard<std::mutex> lock(shared_state_mutex_);
                shared_goal_active_ = false;
                shared_navigation_done_ = true;
                shared_navigation_result_ = BT::NodeStatus::FAILURE;
                return;
            }
            {
                std::lock_guard<std::mutex> lock(shared_state_mutex_);
                shared_goal_handle_ = handle;
            }
            navigation_goal_handle_ = handle;
            RCLCPP_INFO(node_->get_logger(),
                        "ReturnHome[%llu]: goal accepted",
                        static_cast<unsigned long long>(active_goal_id_));
        };
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            const char* result_text = "unknown";
            BT::NodeStatus status = BT::NodeStatus::FAILURE;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    status = BT::NodeStatus::SUCCESS;
                    result_text = "succeeded";
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    result_text = "aborted";
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    result_text = "canceled";
                    break;
                default:
                    result_text = "failed";
                    break;
            }
            navigation_goal_handle_.reset();
            navigation_result_ = status;
            navigation_done_ = true;
            {
                std::lock_guard<std::mutex> lock(shared_state_mutex_);
                shared_goal_handle_.reset();
                shared_goal_active_ = false;
                shared_navigation_done_ = true;
                shared_navigation_result_ = status;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "ReturnHome[%llu]: result=%s",
                        static_cast<unsigned long long>(active_goal_id_),
                        result_text);
        };

    auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "ReturnHome[%llu]: failed to send navigation goal",
                     static_cast<unsigned long long>(active_goal_id_));
        return BT::NodeStatus::FAILURE;
    }

    navigation_goal_handle_ = goal_handle_future.get();
    if (!navigation_goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "ReturnHome[%llu]: navigation goal rejected",
                     static_cast<unsigned long long>(active_goal_id_));
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
                "ReturnHome[%llu]: navigating home to %.2f, %.2f",
                static_cast<unsigned long long>(active_goal_id_),
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnHome::onRunning() {
    if (shared_navigation_done_) {
        navigation_done_ = true;
        navigation_result_ = shared_navigation_result_.load();
        return navigation_result_;
    }
    return BT::NodeStatus::RUNNING;
}

void ReturnHome::onHalted() {
    RCLCPP_INFO(node_->get_logger(),
                "ReturnHome[%llu]: halted",
                static_cast<unsigned long long>(active_goal_id_));
    navigation_goal_handle_.reset();
    navigation_done_ = false;
    navigation_result_ = BT::NodeStatus::RUNNING;
}

bool ReturnHome::sameHomePoint(const geometry_msgs::msg::PoseStamped& a,
                               const geometry_msgs::msg::PoseStamped& b) {
    constexpr double kPosEps = 1e-3;
    constexpr double kOriEps = 1e-3;
    return a.header.frame_id == b.header.frame_id &&
           std::fabs(a.pose.position.x - b.pose.position.x) < kPosEps &&
           std::fabs(a.pose.position.y - b.pose.position.y) < kPosEps &&
           std::fabs(a.pose.orientation.z - b.pose.orientation.z) < kOriEps &&
           std::fabs(a.pose.orientation.w - b.pose.orientation.w) < kOriEps;
}
