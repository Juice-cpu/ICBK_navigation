#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cmath>
#include <memory>
#include <string>

class MultiPointNavigationNode : public rclcpp::Node
{
public:
  MultiPointNavigationNode()
  : Node("multi_point_navigation")
  {
    navigate_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS(10),
      std::bind(&MultiPointNavigationNode::goal_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "multi_point_navigation bridge started: /goal_pose -> navigate_to_pose");
  }

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  static bool same_goal(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    constexpr double pos_eps = 1e-3;
    constexpr double yaw_eps = 1e-3;

    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    const double qz = a.pose.orientation.z - b.pose.orientation.z;
    const double qw = a.pose.orientation.w - b.pose.orientation.w;

    return a.header.frame_id == b.header.frame_id &&
           std::fabs(dx) < pos_eps &&
           std::fabs(dy) < pos_eps &&
           std::fabs(dz) < pos_eps &&
           std::fabs(qz) < yaw_eps &&
           std::fabs(qw) < yaw_eps;
  }

  void send_goal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not ready");
      return;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](GoalHandleNavigateToPose::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "goal rejected by navigate_to_pose");
          return;
        }
        active_goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "goal accepted by navigate_to_pose");
      };
    send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        active_goal_handle_.reset();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "goal aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "goal canceled");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "goal finished with unknown result");
            break;
        }
      };

    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    last_goal_ = goal;
    has_last_goal_ = true;
    RCLCPP_INFO(
      this->get_logger(), "forwarding /goal_pose to navigate_to_pose: (%.2f, %.2f)",
      goal.pose.position.x, goal.pose.position.y);
  }

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (has_last_goal_ && same_goal(*msg, last_goal_)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "received duplicate /goal_pose, skipping resend");
      return;
    }
    send_goal(*msg);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_last_goal_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPointNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
