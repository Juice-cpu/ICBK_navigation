#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class ChaseStateMachine : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ChaseStateMachine() : Node("chase_state_machine"), last_target_time_(now()) {
    declare_parameter<double>("update_distance_threshold", 0.5);
    declare_parameter<double>("replan_period", 0.3);
    declare_parameter<double>("lost_timeout", 1.0);
    declare_parameter<double>("lost_grace", 0.4);
    declare_parameter<std::string>("goal_frame", "odom");
    declare_parameter<double>("goal_offset_x", 0.0);
    declare_parameter<double>("goal_offset_y", 0.0);
    declare_parameter<double>("follow_distance", 2.0);
    declare_parameter<double>("follow_distance_max", 2.0);
    declare_parameter<double>("prediction_horizon", 0.2);
    declare_parameter<double>("smoothing_alpha", 0.7);
    declare_parameter<std::string>("target_topic", "/armor_solver/target");

    update_distance_threshold_ = get_parameter("update_distance_threshold").as_double();
    replan_period_ = get_parameter("replan_period").as_double();
    lost_timeout_ = get_parameter("lost_timeout").as_double();
    lost_grace_ = get_parameter("lost_grace").as_double();
    goal_frame_ = get_parameter("goal_frame").as_string();
    goal_offset_x_ = get_parameter("goal_offset_x").as_double();
    goal_offset_y_ = get_parameter("goal_offset_y").as_double();
    follow_distance_ = get_parameter("follow_distance").as_double();
    follow_distance_max_ = get_parameter("follow_distance_max").as_double();
    prediction_horizon_ = get_parameter("prediction_horizon").as_double();
    smoothing_alpha_ = get_parameter("smoothing_alpha").as_double();
    target_topic_ = get_parameter("target_topic").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_sub_ = create_subscription<rm_interfaces::msg::Target>(
      target_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ChaseStateMachine::targetCallback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    timer_ = create_wall_timer(100ms, std::bind(&ChaseStateMachine::tick, this));

    RCLCPP_INFO(get_logger(), "Chase state machine started.");
  }

 private:
  enum class State { SEARCHING, CHASING, TEMP_LOST, LOST };

  void targetCallback(const rm_interfaces::msg::Target::SharedPtr msg) {
    rm_interfaces::msg::Target target_in_goal = *msg;
    if (!goal_frame_.empty() && msg->header.frame_id != goal_frame_) {
      try {
        geometry_msgs::msg::PoseStamped pose_src;
        pose_src.header = msg->header;
        pose_src.pose.position = msg->position;
        tf2::Quaternion q_src;
        q_src.setRPY(0.0, 0.0, msg->yaw);
        pose_src.pose.orientation = tf2::toMsg(q_src);

        geometry_msgs::msg::PoseStamped pose_out;
        tf_buffer_->transform(pose_src, pose_out, goal_frame_, tf2::durationFromSec(0.2));
        target_in_goal.position = pose_out.pose.position;
        target_in_goal.yaw = tf2::getYaw(pose_out.pose.orientation);

        geometry_msgs::msg::Vector3Stamped vel_src;
        vel_src.header = msg->header;
        vel_src.vector = msg->velocity;
        geometry_msgs::msg::Vector3Stamped vel_out;
        tf_buffer_->transform(vel_src, vel_out, goal_frame_, tf2::durationFromSec(0.2));
        target_in_goal.velocity = vel_out.vector;

        target_in_goal.header.frame_id = goal_frame_;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF transform %s -> %s failed: %s",
          msg->header.frame_id.c_str(), goal_frame_.c_str(), ex.what());
        return;
      }
    }

    last_target_msg_ = target_in_goal;
    last_target_time_ = now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Target msg - tracking:%s pos(%.2f, %.2f) vel(%.2f, %.2f) yaw:%.2f",
      target_in_goal.tracking ? "true" : "false",
      target_in_goal.position.x, target_in_goal.position.y,
      target_in_goal.velocity.x, target_in_goal.velocity.y,
      target_in_goal.yaw);

    state_ = target_in_goal.tracking ? State::CHASING : State::TEMP_LOST;
  }

  void tick() {
    if (!action_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for nav2 action server...");
      return;
    }

    const double since_last_msg = (now() - last_target_time_).seconds();
    if (since_last_msg > lost_timeout_) {
      if (state_ != State::LOST) {
        RCLCPP_WARN(get_logger(), "Target lost, stopping chase.");
      }
      state_ = State::LOST;
    }
    if (state_ == State::TEMP_LOST && since_last_msg > lost_grace_) {
      state_ = State::SEARCHING;
    }

    if (state_ == State::CHASING || state_ == State::TEMP_LOST) {
      sendOrUpdateGoal();
    } else if (state_ == State::LOST && goal_active_) {
      action_client_->async_cancel_all_goals();
      goal_active_ = false;
    }
  }

  void sendOrUpdateGoal() {
    if (!last_target_msg_.tracking) {
      const double since = (now() - last_target_time_).seconds();
      if (since > lost_grace_) {
        return;
      }
    }

    const auto &pos = last_target_msg_.position;
    const auto &vel = last_target_msg_.velocity;
    const double yaw = last_target_msg_.yaw;

    geometry_msgs::msg::Point predicted = pos;
    predicted.x += vel.x * prediction_horizon_;
    predicted.y += vel.y * prediction_horizon_;
    predicted.z += vel.z * prediction_horizon_;

    if (!filtered_goal_initialized_) {
      filtered_goal_position_ = predicted;
      filtered_goal_initialized_ = true;
    } else {
      filtered_goal_position_.x =
        smoothing_alpha_ * predicted.x + (1.0 - smoothing_alpha_) * filtered_goal_position_.x;
      filtered_goal_position_.y =
        smoothing_alpha_ * predicted.y + (1.0 - smoothing_alpha_) * filtered_goal_position_.y;
      filtered_goal_position_.z =
        smoothing_alpha_ * predicted.z + (1.0 - smoothing_alpha_) * filtered_goal_position_.z;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = goal_frame_;

    goal.pose.position.x = filtered_goal_position_.x + goal_offset_x_ -
                           follow_distance_ * std::cos(yaw);
    goal.pose.position.y = filtered_goal_position_.y + goal_offset_y_ -
                           follow_distance_ * std::sin(yaw);
    goal.pose.position.z = filtered_goal_position_.z;

    double rel_dx = goal.pose.position.x - filtered_goal_position_.x;
    double rel_dy = goal.pose.position.y - filtered_goal_position_.y;
    double rel_dist = std::hypot(rel_dx, rel_dy);
    if (rel_dist > follow_distance_max_ && rel_dist > 1e-6) {
      double scale = follow_distance_max_ / rel_dist;
      goal.pose.position.x = filtered_goal_position_.x + rel_dx * scale;
      goal.pose.position.y = filtered_goal_position_.y + rel_dy * scale;
    }

    const double face_yaw =
      std::atan2(filtered_goal_position_.y - goal.pose.position.y,
                 filtered_goal_position_.x - goal.pose.position.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, face_yaw);
    goal.pose.orientation = tf2::toMsg(q);

    double dx = goal.pose.position.x - last_goal_pose_.pose.position.x;
    double dy = goal.pose.position.y - last_goal_pose_.pose.position.y;
    double dist = std::hypot(dx, dy);
    double threshold = update_distance_threshold_;

    auto now_time = now();
    double time_since_last_goal = std::numeric_limits<double>::infinity();
    if (last_goal_send_time_.nanoseconds() != 0) {
      time_since_last_goal = (now_time - last_goal_send_time_).seconds();
    }

    if (!goal_active_ || dist > threshold || time_since_last_goal > replan_period_) {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose = goal;

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandle> handle) {
          if (!handle) {
            RCLCPP_ERROR(get_logger(), "Goal rejected");
            goal_active_ = false;
          } else {
            RCLCPP_INFO(get_logger(), "Goal accepted");
          }
        };
      send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &res) {
          goal_active_ = false;
          switch (res.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "Chase goal reached.");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_WARN(get_logger(), "Chase goal aborted.");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(get_logger(), "Chase goal canceled.");
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Unknown goal result.");
              break;
          }
        };

      action_client_->async_send_goal(goal_msg, send_goal_options);
      last_goal_pose_ = goal;
      goal_active_ = true;
      last_goal_send_time_ = now_time;
      RCLCPP_INFO(get_logger(), "Send chase goal: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    }
  }

  rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  double update_distance_threshold_{0.5};
  double replan_period_{0.3};
  double lost_timeout_{1.0};
  double lost_grace_{0.4};
  std::string goal_frame_{"odom"};
  double goal_offset_x_{0.0};
  double goal_offset_y_{0.0};
  double follow_distance_{2.0};
  double follow_distance_max_{2.0};
  double prediction_horizon_{0.2};
  double smoothing_alpha_{0.7};
  double attack_radius_{2.0};  // unused now
  int attack_num_sectors_{36}; // unused now
  int attack_cost_threshold_{50}; // unused now
  std::string target_topic_{"tracker/target"};

  State state_{State::SEARCHING};
  rm_interfaces::msg::Target last_target_msg_{};
  rclcpp::Time last_target_time_;
  geometry_msgs::msg::PoseStamped last_goal_pose_{};
  bool goal_active_{false};
  rclcpp::Time last_goal_send_time_{};
  geometry_msgs::msg::Point filtered_goal_position_{};
  bool filtered_goal_initialized_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChaseStateMachine>());
  rclcpp::shutdown();
  return 0;
}
