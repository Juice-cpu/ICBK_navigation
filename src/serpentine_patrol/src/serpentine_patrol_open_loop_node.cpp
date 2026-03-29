#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class SerpentinePatrolOpenLoopNode : public rclcpp::Node
{
public:
  SerpentinePatrolOpenLoopNode()
  : Node("serpentine_patrol_open_loop_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    global_frame_candidates_ = this->declare_parameter<std::vector<std::string>>(
      "global_frame_candidates", std::vector<std::string>{"map", "odom"});
    base_frame_candidates_ = this->declare_parameter<std::vector<std::string>>(
      "base_frame_candidates", std::vector<std::string>{"base_link", "base_link_fake"});
    loop_hz_ = this->declare_parameter<double>("loop_hz", 30.0);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.20);
    slow_down_radius_ = this->declare_parameter<double>("slow_down_radius", 1.0);
    waypoint_switch_distance_ = this->declare_parameter<double>("waypoint_switch_distance", 0.45);
    max_speed_ = this->declare_parameter<double>("max_speed", 1.8);
    cruise_speed_ = this->declare_parameter<double>("cruise_speed", 1.6);
    min_speed_ = this->declare_parameter<double>("min_speed", 0.25);
    kp_linear_ = this->declare_parameter<double>("kp_linear", 1.2);
    use_constant_speed_ = this->declare_parameter<bool>("use_constant_speed", true);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 2.0);
    tf_ready_required_cycles_ = this->declare_parameter<int>("tf_ready_required_cycles", 10);
    start_time_ = this->now();
    route_start_time_ = start_time_;
    waypoint_start_time_ = start_time_;

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    init_waypoints();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, loop_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SerpentinePatrolOpenLoopNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Open-loop patrol started, publishing %s, waypoints=%zu",
      cmd_topic_.c_str(), waypoints_.size());
  }

private:
  static std::string join_strings(const std::vector<std::string> & items)
  {
    std::ostringstream oss;
    for (std::size_t i = 0; i < items.size(); ++i) {
      if (i > 0) {
        oss << ",";
      }
      oss << items[i];
    }
    return oss.str();
  }

  static double clamp(const double v, const double lo, const double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  void init_waypoints()
  {
    waypoints_ = {
      // Schematic 6-point serpentine based on the 6 m x 1 m demo layout.
      {0.30, 0.30},
      {5.20, 0.80},
      {0.80, 1.80},
      {5.20, 2.80},
      {0.80, 3.80},
      {5.80, 4.80}
    };
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
  }

  void control_loop()
  {
    if (waypoint_index_ >= waypoints_.size()) {
      publish_stop();
      const double total_elapsed = (this->now() - route_start_time_).seconds();
      RCLCPP_INFO(this->get_logger(), "All waypoints reached. Total elapsed: %.2f s. Stop.", total_elapsed);
      rclcpp::shutdown();
      return;
    }

    if (!motion_started_) {
      const double elapsed = (this->now() - start_time_).seconds();
      if (elapsed < startup_delay_sec_) {
        publish_stop();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Waiting startup delay: %.1f / %.1f s", elapsed, startup_delay_sec_);
        return;
      }
    }

    geometry_msgs::msg::TransformStamped tf_map_to_base;
    bool got_tf = false;
    std::string active_global_frame;
    std::string active_base_frame;

    for (const auto & global_frame : global_frame_candidates_) {
      for (const auto & base_frame : base_frame_candidates_) {
        try {
          tf_map_to_base = tf_buffer_.lookupTransform(
            global_frame, base_frame, tf2::TimePointZero);
          active_global_frame = global_frame;
          active_base_frame = base_frame;
          got_tf = true;
          break;
        } catch (const tf2::TransformException &) {
        }
      }
      if (got_tf) {
        break;
      }
    }

    if (!got_tf) {
      tf_ready_count_ = 0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF lookup failed for all global frames [%s] and base frames [%s]",
        join_strings(global_frame_candidates_).c_str(),
        join_strings(base_frame_candidates_).c_str());
      publish_stop();
      return;
    }

    if (active_global_frame != global_frame_candidates_.front() ||
      active_base_frame != base_frame_candidates_.front())
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Using fallback TF %s->%s",
        active_global_frame.c_str(), active_base_frame.c_str());
    }

    if (!motion_started_) {
      tf_ready_count_++;
      if (tf_ready_count_ < tf_ready_required_cycles_) {
        publish_stop();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Waiting TF ready cycles: %d / %d", tf_ready_count_, tf_ready_required_cycles_);
        return;
      }
      motion_started_ = true;
      route_start_time_ = this->now();
      waypoint_start_time_ = route_start_time_;
      RCLCPP_INFO(this->get_logger(), "Startup gate passed. Begin motion.");
    }

    const double rx = tf_map_to_base.transform.translation.x;
    const double ry = tf_map_to_base.transform.translation.y;
    const double yaw = tf2::getYaw(tf_map_to_base.transform.rotation);

    const double gx = waypoints_[waypoint_index_].first;
    const double gy = waypoints_[waypoint_index_].second;

    const double dx = gx - rx;
    const double dy = gy - ry;
    const double dist = std::hypot(dx, dy);

    const bool is_last_waypoint = (waypoint_index_ + 1 >= waypoints_.size());
    const double reach_distance = is_last_waypoint ? goal_tolerance_ : waypoint_switch_distance_;

    if (dist < reach_distance) {
      const double wp_elapsed = (this->now() - waypoint_start_time_).seconds();
      const double total_elapsed = (this->now() - route_start_time_).seconds();
      RCLCPP_INFO(
        this->get_logger(),
        "Reached waypoint %zu at (%.2f, %.2f), dist=%.3f, waypoint time=%.2f s, total=%.2f s",
        waypoint_index_, gx, gy, dist, wp_elapsed, total_elapsed);
      waypoint_index_++;
      waypoint_start_time_ = this->now();
      return;
    }

    // Convert map-frame position error into base_link frame command.
    const double ex = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ey = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    double vx = 0.0;
    double vy = 0.0;
    if (use_constant_speed_) {
      // Keep constant speed magnitude across the route and only change heading.
      const double e_norm = std::hypot(ex, ey);
      const double speed = clamp(cruise_speed_, 0.0, max_speed_);
      if (e_norm > 1e-6) {
        vx = speed * ex / e_norm;
        vy = speed * ey / e_norm;
      }
    } else {
      vx = kp_linear_ * ex;
      vy = kp_linear_ * ey;

      double max_cmd_speed = max_speed_;
      if (is_last_waypoint && dist < slow_down_radius_) {
        max_cmd_speed = std::max(min_speed_, max_speed_ * dist / slow_down_radius_);
      }

      const double cmd_norm = std::hypot(vx, vy);
      if (cmd_norm > max_cmd_speed && cmd_norm > 1e-6) {
        const double scale = max_cmd_speed / cmd_norm;
        vx *= scale;
        vy *= scale;
      }
    }

    vx = clamp(vx, -max_speed_, max_speed_);
    vy = clamp(vy, -max_speed_, max_speed_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string cmd_topic_;
  std::vector<std::string> global_frame_candidates_;
  std::vector<std::string> base_frame_candidates_;
  double loop_hz_{30.0};
  double goal_tolerance_{0.20};
  double slow_down_radius_{1.0};
  double waypoint_switch_distance_{0.45};
  double max_speed_{1.8};
  double cruise_speed_{1.6};
  double min_speed_{0.25};
  double kp_linear_{1.2};
  bool use_constant_speed_{true};
  double startup_delay_sec_{2.0};
  int tf_ready_required_cycles_{10};
  rclcpp::Time start_time_;
  rclcpp::Time route_start_time_;
  rclcpp::Time waypoint_start_time_;
  int tf_ready_count_{0};
  bool motion_started_{false};

  std::vector<std::pair<double, double>> waypoints_;
  std::size_t waypoint_index_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerpentinePatrolOpenLoopNode>());
  rclcpp::shutdown();
  return 0;
}
