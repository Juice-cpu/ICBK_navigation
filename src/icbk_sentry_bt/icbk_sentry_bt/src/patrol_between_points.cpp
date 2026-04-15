#include "patrol_between_points.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
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
      goal_topic_("goal_pose"),
      map_frame_("map"),
      robot_frame_("base_link"),
      hp_switch_threshold_(300),
      hp_abort_threshold_(200),
      target_timeout_(0.5),
      position_tolerance_(0.25),
      yaw_tolerance_(0.35),
      republish_interval_(0.5),
      has_target_(false),
      target_active_(false),
      last_target_time_ns_(0),
      has_current_goal_(false)
{
    if (!node_) {
        rclcpp::NodeOptions options;
        options.use_global_arguments(false);
        node_ = rclcpp::Node::make_shared("patrol_between_points_node", options);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

BT::PortsList PatrolBetweenPoints::providedPorts()
{
    return {
        BT::InputPort<std::string>("high_points", "", "High HP patrol points, split by | with x;y;yaw"),
        BT::InputPort<std::string>("low_points", "", "Mid HP patrol points, split by | with x;y;yaw"),
        BT::InputPort<int>("hp_switch_threshold", 300, "HP threshold for switching to high route"),
        BT::InputPort<int>("hp_abort_threshold", 200, "HP threshold below which patrol must yield"),
        BT::InputPort<std::string>("target_topic", "/auto_aim_target_pos", "Target topic to monitor"),
        BT::InputPort<double>("target_timeout", 0.5, "Seconds to consider target valid"),
        BT::InputPort<std::string>("topic_name", "goal_pose", "Goal topic to publish"),
        BT::InputPort<double>("position_tolerance", 0.25, "Goal position tolerance in meters"),
        BT::InputPort<double>("yaw_tolerance", 0.35, "Goal yaw tolerance in radians"),
        BT::InputPort<double>("republish_interval", 0.5, "Goal republish interval in seconds"),
        BT::InputPort<std::string>("map_frame", "map", "Global frame"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot base frame"),
        BT::InputPort<SentryRefereeReceiver*>("key_port")
    };
}

rclcpp::Node::SharedPtr PatrolBetweenPoints::shared_node()
{
    return node_;
}

void PatrolBetweenPoints::cleanupSharedResources()
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

bool PatrolBetweenPoints::ensurePoseAvailable()
{
    geometry_msgs::msg::PoseStamped pose;
    return currentRobotPose(pose);
}

bool PatrolBetweenPoints::parseGoalToken(const std::string& token, geometry_msgs::msg::PoseStamped& pose)
{
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

bool PatrolBetweenPoints::parseRoute(const std::string& route_str, std::vector<geometry_msgs::msg::PoseStamped>& route)
{
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

bool PatrolBetweenPoints::loadInputs()
{
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
        goal_topic_ = getInput<std::string>("topic_name").value();
        position_tolerance_ = getInput<double>("position_tolerance").value();
        yaw_tolerance_ = getInput<double>("yaw_tolerance").value();
        republish_interval_ = getInput<double>("republish_interval").value();
        map_frame_ = getInput<std::string>("map_frame").value();
        robot_frame_ = getInput<std::string>("robot_frame").value();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool PatrolBetweenPoints::routeShouldBeHigh() const
{
    if (!ref_ptr_) {
        return false;
    }
    const auto data = ref_ptr_->getLatestData();
    return data.sentry_hp >= hp_switch_threshold_;
}

bool PatrolBetweenPoints::refreshRoute(bool force_reset)
{
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

void PatrolBetweenPoints::updateRouteForCurrentHp()
{
    const bool should_use_high = routeShouldBeHigh();
    if (should_use_high == using_high_route_) {
        return;
    }
    if (!refreshRoute(true)) {
        yield_requested_ = true;
        navigation_done_ = true;
        navigation_result_ = BT::NodeStatus::FAILURE;
        return;
    }
    restart_requested_ = true;
}

bool PatrolBetweenPoints::shouldYieldForHp() const
{
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

bool PatrolBetweenPoints::targetCurrentlyVisible() const
{
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

void PatrolBetweenPoints::startMonitorLoop()
{
    stopMonitorLoop();
    stop_monitor_ = false;
    patrol_active_ = true;
    monitor_thread_ = std::thread([this]() { monitorLoop(); });
}

void PatrolBetweenPoints::stopMonitorLoop()
{
    stop_monitor_ = true;
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void PatrolBetweenPoints::monitorLoop()
{
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
                break;
            }
            sendNextGoal();
        }

        updateRouteForCurrentHp();
        std::this_thread::sleep_for(20ms);
    }
}

bool PatrolBetweenPoints::publishGoal(const geometry_msgs::msg::PoseStamped& pose, bool force_log)
{
    if (!publishers_.count(goal_topic_)) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        publishers_[goal_topic_] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, qos);
    }

    auto msg = pose;
    msg.header.stamp = node_->now();
    msg.header.frame_id = map_frame_;
    publishers_[goal_topic_]->publish(msg);
    last_goal_publish_time_ = msg.header.stamp;

    if (force_log) {
        RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: published patrol goal on %s", goal_topic_.c_str());
    }
    return true;
}

bool PatrolBetweenPoints::currentRobotPose(geometry_msgs::msg::PoseStamped& pose) const
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
            "PatrolBetweenPoints: TF %s -> %s failed: %s",
            map_frame_.c_str(), robot_frame_.c_str(), ex.what());
        return false;
    }

    pose.header = transform.header;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
}

double PatrolBetweenPoints::normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool PatrolBetweenPoints::currentGoalReached() const
{
    if (!has_current_goal_) {
        return false;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!currentRobotPose(robot_pose)) {
        return false;
    }

    const double dx = current_goal_.pose.position.x - robot_pose.pose.position.x;
    const double dy = current_goal_.pose.position.y - robot_pose.pose.position.y;
    const double distance = std::hypot(dx, dy);

    tf2::Quaternion goal_q;
    tf2::fromMsg(current_goal_.pose.orientation, goal_q);
    double goal_roll = 0.0;
    double goal_pitch = 0.0;
    double goal_yaw = 0.0;
    tf2::Matrix3x3(goal_q).getRPY(goal_roll, goal_pitch, goal_yaw);

    tf2::Quaternion robot_q;
    tf2::fromMsg(robot_pose.pose.orientation, robot_q);
    double robot_roll = 0.0;
    double robot_pitch = 0.0;
    double robot_yaw = 0.0;
    tf2::Matrix3x3(robot_q).getRPY(robot_roll, robot_pitch, robot_yaw);

    const double yaw_error = std::fabs(normalizeAngle(goal_yaw - robot_yaw));
    return distance <= position_tolerance_ && yaw_error <= yaw_tolerance_;
}

BT::NodeStatus PatrolBetweenPoints::sendNextGoal()
{
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
        current_goal_ = next_pose;
        has_current_goal_ = true;
    }

    if (!publishGoal(next_pose, true)) {
        navigation_result_ = BT::NodeStatus::FAILURE;
        navigation_done_ = true;
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: navigating to route[%zu] %.2f, %.2f",
                current_index, next_pose.pose.position.x, next_pose.pose.position.y);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatrolBetweenPoints::onStart()
{
    if (!loadInputs()) {
        RCLCPP_ERROR(node_->get_logger(), "PatrolBetweenPoints: failed to load inputs");
        return BT::NodeStatus::FAILURE;
    }
    if (!ensurePoseAvailable()) {
        RCLCPP_WARN(node_->get_logger(), "PatrolBetweenPoints: robot pose unavailable, cannot start patrol");
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
    has_current_goal_ = false;

    startMonitorLoop();
    if (paused_for_target_) {
        return BT::NodeStatus::RUNNING;
    }
    return sendNextGoal();
}

BT::NodeStatus PatrolBetweenPoints::onRunning()
{
    if (yield_requested_) {
        return BT::NodeStatus::FAILURE;
    }
    if (restart_requested_) {
        restart_requested_ = false;
        return sendNextGoal();
    }
    if (!paused_for_target_ && has_current_goal_ && currentGoalReached()) {
        return sendNextGoal();
    }
    if (!paused_for_target_ && has_current_goal_ && republish_interval_ > 0.0 &&
        (node_->now() - last_goal_publish_time_).seconds() >= republish_interval_) {
        publishGoal(current_goal_, false);
    }
    if (navigation_done_) {
        return navigation_result_;
    }
    return BT::NodeStatus::RUNNING;
}

void PatrolBetweenPoints::onHalted()
{
    patrol_active_ = false;
    stopMonitorLoop();
    yield_requested_ = false;
    restart_requested_ = false;
    navigation_done_ = true;
    navigation_result_ = BT::NodeStatus::FAILURE;
    paused_for_target_ = false;
    has_current_goal_ = false;
    target_sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "PatrolBetweenPoints: halted");
}
