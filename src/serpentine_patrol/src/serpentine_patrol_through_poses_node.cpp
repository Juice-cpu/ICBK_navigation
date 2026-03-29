#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class SerpentinePatrolThroughPosesNode : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  SerpentinePatrolThroughPosesNode()
  : Node("serpentine_patrol_through_poses_node")
  {
    RCLCPP_INFO(this->get_logger(), "Serpentine ThroughPoses node started.");

    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");

    initialize_waypoints();

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SerpentinePatrolThroughPosesNode::wait_and_send_goal, this));
  }

private:
  geometry_msgs::msg::PoseStamped make_pose(const double x, const double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void apply_heading_along_path()
  {
    if (waypoints_.size() < 2) {
      return;
    }

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const size_t next = (i + 1 < waypoints_.size()) ? i + 1 : i;
      const double dx = waypoints_[next].pose.position.x - waypoints_[i].pose.position.x;
      const double dy = waypoints_[next].pose.position.y - waypoints_[i].pose.position.y;
      const double yaw = std::atan2(dy, dx);
      const double half_yaw = yaw * 0.5;
      waypoints_[i].pose.orientation.w = std::cos(half_yaw);
      waypoints_[i].pose.orientation.z = std::sin(half_yaw);
    }
  }

  void initialize_waypoints()
  {
    waypoints_.clear();

    const std::vector<std::pair<double, double>> pts = {
      {1.59, -0.78},
      {2.64, 0.09},
      {1.82, 1.57},
      {3.27, 1.89},
      {3.17, 0.44},
      {3.45, -0.96},
      {4.46, -0.15},
      {3.62, 1.03},
      {4.78, 1.93},
      {5.26, 0.46}
    };

    for (const auto & p : pts) {
      waypoints_.push_back(make_pose(p.first, p.second));
    }

    apply_heading_along_path();

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      RCLCPP_INFO(
        this->get_logger(), "Waypoint %zu: (%.2f, %.2f)",
        i, waypoints_[i].pose.position.x, waypoints_[i].pose.position.y);
    }
  }

  void wait_and_send_goal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for navigate_through_poses action server...");
      return;
    }

    timer_->cancel();
    start_time_ = std::chrono::steady_clock::now();
    const auto start_wall_time = std::chrono::system_clock::now();
    const auto time_t = std::chrono::system_clock::to_time_t(start_wall_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    RCLCPP_INFO(this->get_logger(), "=== ThroughPoses started at: %s ===", ss.str().c_str());

    send_goal();
  }

  void send_goal()
  {
    if (waypoints_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No waypoints configured.");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = waypoints_;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending NavigateThroughPoses with %zu waypoints.",
      goal_msg.poses.size());

    auto send_goal_options =
      rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(
      &SerpentinePatrolThroughPosesNode::goal_response_callback, this,
      std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(
      &SerpentinePatrolThroughPosesNode::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(
      &SerpentinePatrolThroughPosesNode::result_callback, this,
      std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavigateThroughPoses goal rejected by server.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "NavigateThroughPoses goal accepted.");
  }

  void feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> /*feedback*/)
  {
  }

  void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
  {
    const auto end_time = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(
          this->get_logger(),
          "NavigateThroughPoses succeeded. Total elapsed time: %.3f s",
          elapsed.count() / 1000.0);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(
          this->get_logger(),
          "NavigateThroughPoses aborted. Total elapsed time: %.3f s",
          elapsed.count() / 1000.0);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(
          this->get_logger(),
          "NavigateThroughPoses canceled. Total elapsed time: %.3f s",
          elapsed.count() / 1000.0);
        break;
      default:
        RCLCPP_ERROR(
          this->get_logger(),
          "NavigateThroughPoses finished with unknown result code. Total elapsed time: %.3f s",
          elapsed.count() / 1000.0);
        break;
    }

    rclcpp::shutdown();
  }

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerpentinePatrolThroughPosesNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
