#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <vector>
#include <string>

class MultiPointNavigationNode : public rclcpp::Node
{
public:
  MultiPointNavigationNode()
  : Node("multi_point_navigation")
  {
    // 获取参数
    this->declare_parameter<std::string>("navigation_mode", "single_point");
     this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});

    // 创建 Action Client
    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // 创建 Goal Pose 订阅者
    goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&MultiPointNavigationNode::goal_pose_callback, this, std::placeholders::_1));

    // 设置定时器用于巡逻模式
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MultiPointNavigationNode::timer_callback, this));

    // 初始化目标点
    create_waypoints();

    // 获取配置的导航模式
    navigation_mode_ = this->get_parameter("navigation_mode").as_string();
    RCLCPP_INFO(this->get_logger(), "Navigation mode: %s", navigation_mode_.c_str());

    if (navigation_mode_ == "patrol") {
      patrol_mode_ = true;
      current_waypoint_index_ = 0;
    } else if (navigation_mode_ == "sequence") {
      sequence_mode_ = true;
      current_waypoint_index_ = 0;
    } else {
      // 默认单点模式
      single_point_mode_ = true;
    }
  }

private:
  // 创建目标点（单点或多点）
  void create_waypoints()
  {
    // 这里可以通过从参数获取目标点
    geometry_msgs::msg::PoseStamped point_A;
    point_A.header.frame_id = "map";
    point_A.pose.position.x = 1.0;
    point_A.pose.position.y = 2.0;
    point_A.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped point_B;
    point_B.header.frame_id = "map";
    point_B.pose.position.x = -2.0;
    point_B.pose.position.y = -3.0;
    point_B.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped point_C;
    point_C.header.frame_id = "map";
    point_C.pose.position.x = 3.0;
    point_C.pose.position.y = 3.0;
    point_C.pose.orientation.w = 1.0;

    waypoints_.push_back(point_A);
    waypoints_.push_back(point_B);
    waypoints_.push_back(point_C);
  }

  // 发布目标点并发送给导航服务器
  void send_goal(const geometry_msgs::msg::PoseStamped& goal)
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal;

    // 如果没有连接到 Action Server，等待服务器准备好
    if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
      return;
    }

    // 发送目标
    auto send_goal_future = navigate_to_pose_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Navigating to (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
  }

  // 定时器回调函数，用于巡逻模式
  void timer_callback()
  {
    if (patrol_mode_) {
      if (current_waypoint_index_ >= waypoints_.size()) {
        current_waypoint_index_ = 0; // 循环
      }
      const auto& current_goal = waypoints_[current_waypoint_index_];
      send_goal(current_goal);
      current_waypoint_index_++;
    } else if (sequence_mode_) {
      // 顺序模式
      if (current_waypoint_index_ >= waypoints_.size()) {
        current_waypoint_index_ = 0; // 重置顺序
      }
      const auto& current_goal = waypoints_[current_waypoint_index_];
      send_goal(current_goal);
      current_waypoint_index_++;
    } else if (single_point_mode_) {
      // 单点模式
      if (!waypoints_.empty()) {
        send_goal(waypoints_[0]);  // 默认第一个点
      }
    }
  }

  // 目标位置话题回调函数
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal pose: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
    send_goal(*msg);
  }

  // Action Client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;

  // 目标点订阅器
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;

  // 目标点列表
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_waypoint_index_ = 0;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 导航模式
  std::string navigation_mode_;
  bool patrol_mode_ = false;
  bool sequence_mode_ = false;
  bool single_point_mode_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPointNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
