#include "patrol_between_points.h"

#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <cstdlib>
#include <utility>

using namespace std::chrono_literals;

PatrolBetweenPoints::PatrolBetweenPoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      navigation_done_(false),
      navigation_result_(BT::NodeStatus::RUNNING),
      patrol_active_(false),
      stop_monitor_(false),
      yield_requested_(false),
      restart_requested_(false),
      paused_for_target_(false),
      route_index_(0),
      using_high_route_(false),
      ref_ptr_(nullptr),
      hp_switch_threshold_(300),
      hp_abort_threshold_(200),
      target_timeout_(0.5),
      has_target_(false),
      target_active_(false),
      last_target_time_ns_(0) {
    if (!node_) {
        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        node_ = rclcpp::Node::make_shared("patrol_between_points_node", options);
        navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    if (!executor_) {
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

BT::PortsList PatrolBetweenPoints::providedPorts() {
    return {
        BT::InputPort<std::string>("high_points", "", "High HP patrol points, split by | with x;y;yaw"),
        BT::InputPort<std::string>("low_points", "", "Mid HP patrol points, split by | with x;y;yaw"),
        BT::InputPort<int>("hp_switch_threshold", 300, "HP threshold for switching to high route"),
        BT::InputPort<int>("hp_abort_threshold", 200, "HP threshold below which patrol must yield"),
        BT::InputPort<std::string>("target_topic", "/auto_aim_target_pos", "Target topic to monitor"),
        BT::InputPort<double>("target_timeout", 0.5, "Seconds to consider target valid"),
        BT::InputPort<SentryRefereeReceiver*>("key_port")
    };
}

rclcpp::Node::SharedPtr PatrolBetweenPoints::shared_node() {
    return node_;
}

void PatrolBetweenPoints::cleanupSharedResources() {
    executor_running_ = false;
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    executor_.reset();
    navigation_client_.reset();
    node_.reset();
}

bool PatrolBetweenPoints::ensureActionServer() {
    if (!navigation_client_) {
        return false;
    }
    if (!navigation_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(node_->get_logger(), "PatrolBetweenPoints: waiting for NavigateToPose action server");
        return false;
    }
    return true;
}

bool PatrolBetweenPoints::cancelExistingGoals() {
    if (!navigation_client_) {
        return false;
    }
    auto cancel_future = navigation_client_->async_cancel_all_goals();
    const auto status = cancel_future.wait_for(1s);
    if (status != std::future_status::ready) {
        RCLCPP_WARN(node_->get_logger(), "PatrolBetweenPoints: timeout while canceling existing goals");
        return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    navigation_goal_handle_.reset();
    return true;
}

void PatrolBetweenPoints::cancelCurrentGoal(const char* reason) {
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!navigation_client_ || !navigation_goal_handle_) {
            return;
        }
        goal_handle = navigation_goal_handle_;
        navigation_goal_handle_.reset();
    }

    RCLCPP_INFO(node_->get_logger(),
                "PatrolBetweenPoints: cancelCurrentGoal reason=%s",
                reason ? reason : "unknown");
    try {
        navigation_client_->async_cancel_goal(goal_handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError& ex) {
        RCLCPP_WARN(node_->get_logger(),
                    "PatrolBetweenPoints: cancel ignored for unknown goal handle: %s",
                    ex.what());
    }
}

bool PatrolBetweenPoints::parseGoalToken(const std::string& token, geometry_msgs::msg::PoseStamped& pose) {
    std::stringstream ss(token);
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

    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return true;
}

bool PatrolBetweenPoints::parseRoute(const std::string& route_str, std::vector<geometry_msgs::msg::PoseStamped>& route) {
    route.clear();
    std::stringstream ss(route_str);
    std::string token;
    while (std::getline(ss, token, '|')) {
        if (token.empty()) {
            continue;
        }
        geometry_msgs::msg::PoseStamped pose;
        if (!parseGoalToken(token, pose)) {
            return false;
        }
        route.push_back(pose);
    }
    return !route.empty();
}

bool PatrolBetweenPoints::loadInputs() {
    try {
        auto ref = getInput<SentryRefereeReceiver*>("key_port");
        if (!ref) {
            return false;
        }
        ref_ptr_ = *ref;
        high_points_ = getInput<std::string>("high_points").value();
        low_points_ = getInput<std::string>("low_points").value();
        hp_switch_threshold_ = getInput<int>("hp_switch_threshold").value();
        hp_abort_threshold_ = getInput<int>("hp_abort_threshold").value();
        target_topic_ = getInput<std::string>("target_topic").value();
        target_timeout_ = getInput<double>("target_timeout").value();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool PatrolBetweenPoints::routeShouldBeHigh() const {
    if (!ref_ptr_) {
        return false;
    }
    const auto data = ref_ptr_->getLatestData();
    return data.sentry_hp >= hp_switch_threshold_;
}

bool PatrolBetweenPoints::refreshRoute(bool force_reset) {
    const bool should_use_high = routeShouldBeHigh();
    if (!force_reset && should_use_high == using_high_route_) {
        return true;
    }

    std::vector<geometry_msgs::msg::PoseStamped> route;
    const std::string& route_str = should_use_high ? high_points_ : low_points_;
    if (!parseRoute(route_str, route)) {
        RCLCPP_ERROR(node_->get_logger(), "PatrolBetweenPoints: invalid route string: %s", route_str.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_route_ = std::move(route);
        route_index_ = 0;
        using_high_route_ = should_use_high;
    }

    const auto data = ref_ptr_->getLatestData();
    RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: switched to %s route with %zu points (hp=%u)",
                using_high_route_ ? "high" : "mid",
                active_route_.size(),
                static_cast<unsigned>(data.sentry_hp));
    return true;
}

void PatrolBetweenPoints::updateRouteForCurrentHp() {
    const bool should_use_high = routeShouldBeHigh();
    if (should_use_high == using_high_route_) {
        return;
    }
    if (!refreshRoute(true)) {
        yield_requested_ = true;
        navigation_done_ = true;
        navigation_result_ = BT::NodeStatus::FAILURE;
        cancelCurrentGoal("route_switch");
        return;
    }
    restart_requested_ = true;
    cancelCurrentGoal("route_switch");
}

bool PatrolBetweenPoints::shouldYieldForHp() const {
    if (!ref_ptr_) {
        return true;
    }
    const auto data = ref_ptr_->getLatestData();
    if (data.sentry_hp < hp_abort_threshold_) {
        RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: HP=%u below abort threshold %d, yielding patrol",
                    static_cast<unsigned>(data.sentry_hp), hp_abort_threshold_);
        return true;
    }
    return false;
}

bool PatrolBetweenPoints::targetCurrentlyVisible() const {
    if (!has_target_) {
        return false;
    }
    const int64_t last_ns = last_target_time_ns_.load();
    if (last_ns == 0 || !target_active_) {
        return false;
    }
    const auto age = (node_->now() - rclcpp::Time(last_ns, node_->get_clock()->get_clock_type())).seconds();
    return age <= target_timeout_;
}
/// @brief 
void PatrolBetweenPoints::startMonitorLoop() {
    stopMonitorLoop();
    stop_monitor_ = false;
    patrol_active_ = true;
    monitor_thread_ = std::thread([this]() { monitorLoop(); });
}

void PatrolBetweenPoints::stopMonitorLoop() {
    stop_monitor_ = true;
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void PatrolBetweenPoints::monitorLoop() {
    while (!stop_monitor_) {
        if (!patrol_active_) {
            std::this_thread::sleep_for(20ms);
            continue;
        }

        if (shouldYieldForHp()) {
            yield_requested_ = true;
            navigation_done_ = true;
            navigation_result_ = BT::NodeStatus::FAILURE;
            break;
        }

        if (targetCurrentlyVisible()) {
            if (!paused_for_target_) {
                RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: target visible, pausing patrol");
                paused_for_target_ = true;
                cancelCurrentGoal("target_visible");
            }
            std::this_thread::sleep_for(20ms);
            continue;
        }

        if (paused_for_target_) {
            paused_for_target_ = false;
            restart_requested_ = false;
            yield_requested_ = false;
            navigation_done_ = false;
            navigation_result_ = BT::NodeStatus::RUNNING;
            if (!refreshRoute(true)) {
                yield_requested_ = true;
                navigation_done_ = true;
                navigation_result_ = BT::NodeStatus::FAILURE;
                cancelCurrentGoal("route_refresh_failed");
                break;
            }
            // Resume patrol immediately from monitor loop (not relying on BT tick or action callbacks).
            sendNextGoal();
        }

        updateRouteForCurrentHp();
        std::this_thread::sleep_for(20ms);
    }
}

BT::NodeStatus PatrolBetweenPoints::sendNextGoal() {
    geometry_msgs::msg::PoseStamped next_pose;
    std::size_t current_index = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_route_.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        if (route_index_ >= active_route_.size()) {
            route_index_ = 0;
        }
        current_index = route_index_;
        next_pose = active_route_[route_index_];
        route_index_ = (route_index_ + 1) % active_route_.size();
        navigation_done_ = false;
        navigation_result_ = BT::NodeStatus::RUNNING;
        navigation_goal_handle_.reset();
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = next_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!handle) {
                RCLCPP_ERROR(node_->get_logger(), "PatrolBetweenPoints: goal rejected");
                navigation_result_ = BT::NodeStatus::FAILURE;
                navigation_done_ = true;
                return;
            }
            navigation_goal_handle_ = handle;
        };
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                navigation_goal_handle_.reset();
            }
            if (!patrol_active_) {
                return;
            }

            if (yield_requested_) {
                navigation_result_ = BT::NodeStatus::FAILURE;
                navigation_done_ = true;
                return;
            }

            if (paused_for_target_) {
                return;
            }

            if (restart_requested_) {
                restart_requested_ = false;
                sendNextGoal();
                return;
            }

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                sendNextGoal();
                return;
            }

            navigation_result_ = BT::NodeStatus::FAILURE;
            navigation_done_ = true;
        };

    navigation_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: navigating to route[%zu] %.2f, %.2f",
                current_index, next_pose.pose.position.x, next_pose.pose.position.y);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatrolBetweenPoints::onStart() {
    if (!ensureActionServer()) {
        return BT::NodeStatus::FAILURE;
    }
    if (!loadInputs()) {
        RCLCPP_ERROR(node_->get_logger(), "PatrolBetweenPoints: failed to load inputs");
        return BT::NodeStatus::FAILURE;
    }

    has_target_ = false;
    target_active_ = false;
    last_target_time_ns_ = 0;
    target_sub_ = node_->create_subscription<std_msgs::msg::String>(
        target_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            const char* raw = msg->data.c_str();
            char* end = nullptr;
            const double flag = std::strtod(raw, &end);
            target_active_ = (end != raw) && (flag > 0.5);
            has_target_ = true;
            last_target_time_ns_ = node_->now().nanoseconds();
        });

    const bool hp_yield = shouldYieldForHp();
    const bool target_visible = targetCurrentlyVisible();
    if (hp_yield) {
        return BT::NodeStatus::FAILURE;
    }
    paused_for_target_ = target_visible;
    if (!refreshRoute(true)) {
        return BT::NodeStatus::FAILURE;
    }

    yield_requested_ = false;
    restart_requested_ = false;
    navigation_done_ = false;
    navigation_result_ = BT::NodeStatus::RUNNING;
    patrol_active_ = true;

    cancelExistingGoals();
    startMonitorLoop();
    if (paused_for_target_) {
        return BT::NodeStatus::RUNNING;
    }
    return sendNextGoal();
}

BT::NodeStatus PatrolBetweenPoints::onRunning() {
    if (yield_requested_) {
        return BT::NodeStatus::FAILURE;
    }
    if (navigation_done_) {
        return navigation_result_;
    }
    return BT::NodeStatus::RUNNING;
}

void PatrolBetweenPoints::onHalted() {
    patrol_active_ = false;
    stopMonitorLoop();
    cancelCurrentGoal("halt");
    navigation_done_ = true;
    navigation_result_ = BT::NodeStatus::FAILURE;
    paused_for_target_ = false;
    RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: halted");
}
