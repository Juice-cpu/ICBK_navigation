#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace goal_approach_controller
{

class GoalApproachController : public nav2_core::Controller
{
public:
  GoalApproachController() = default;
  ~GoalApproachController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Unable to lock lifecycle node in GoalApproachController");
    }

    logger_ = node->get_logger();

    // 这个包装器把常规路径跟踪交给内部 Nav2 控制器处理。
    // 这里默认内部控制器用 MPPI，再额外叠加“接近目标点时减速/直驱贴点”的逻辑。
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_distance",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_kp",
      rclcpp::ParameterValue(1.0));

    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);

    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController wrapping [%s], approach_distance=%.2f, approach_velocity=%.2f, "
      "direct_approach_distance=%.2f, direct_approach_kp=%.2f",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,
      direct_approach_distance_, direct_approach_kp_);
  }

  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  void activate() override
  {
    inner_controller_->activate();
  }

  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
    }
    inner_controller_->setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    const double dx = goal_.pose.position.x - pose.pose.position.x;
    const double dy = goal_.pose.position.y - pose.pose.position.y;
    const double dist = std::hypot(dx, dy);

    if (dist < direct_approach_distance_) {
      // 进入最终贴点半径后，绕过 MPPI 可能给出的弧线速度，
      // 直接朝目标点给一个受限的小平移速度。
      const double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
      if (dist > 0.01) {
        cmd.twist.linear.x = target_speed * (dx / dist);
        cmd.twist.linear.y = target_speed * (dy / dist);
      } else {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
      }
      cmd.twist.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      // 在更大的接近区域内，保留 MPPI 给出的运动方向，
      // 但限制合速度，减少高速到点时的冲过头现象。
      const double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
      if (speed > approach_velocity_) {
        const double scale = approach_velocity_ / speed;
        cmd.twist.linear.x *= scale;
        cmd.twist.linear.y *= scale;
        cmd.twist.angular.z *= scale;
      }
    }

    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }

private:
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  geometry_msgs::msg::PoseStamped goal_;
  double approach_distance_{1.5};
  double approach_velocity_{0.5};
  double direct_approach_distance_{0.5};
  double direct_approach_kp_{1.0};
};

}  // namespace goal_approach_controller

PLUGINLIB_EXPORT_CLASS(
  goal_approach_controller::GoalApproachController,
  nav2_core::Controller)
