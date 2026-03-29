#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// 定义状态枚举
enum class PatrolState {
    SENDING_GOAL,  // 正在发送目标点
    WAITING_RESULT, // 等待Nav2到达目标点
    GOING_HOME,    // 巡逻结束，正在回家
    IDLE           // 所有任务完成，空闲
};

class PatrolDriver : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    PatrolDriver() : Node("patroldriver_node"), current_point_(0), patrol_count_(0), state_(PatrolState::SENDING_GOAL)
    {
        RCLCPP_INFO(this->get_logger(), "Patrol driver started. Using Nav2 for navigation.");

        // 创建Nav2的Action客户端
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 订阅里程计，获取机器人当前位置（可选，但用于日志很有用）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PatrolDriver::odom_callback, this, std::placeholders::_1));

        // 使用定时器来触发逻辑，而不是直接在回调里
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&PatrolDriver::control_loop, this));

        // 定义巡逻点
        points_[0].header.frame_id = "map";
        points_[0].pose.position.x = 1.10;
        points_[0].pose.position.y = 1.73;
        points_[0].pose.orientation.w = 1.0;

        points_[1].header.frame_id = "map";
        points_[1].pose.position.x = 1.73;
        points_[1].pose.position.y = 1.13;
        points_[1].pose.orientation.w = 1.0;

        // 定义家的坐标
        home_pose_.header.frame_id = "map";
        home_pose_.pose.position.x = 0.11;
        home_pose_.pose.position.y = 0.01;
        home_pose_.pose.orientation.w = 1.0;
    }

private:
    void control_loop()
    {
        // 等待Nav2 Action服务上线
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server to be available...");
            return;
        }

        switch (state_)
        {
        case PatrolState::SENDING_GOAL:
            send_goal(points_[current_point_]);
            state_ = PatrolState::WAITING_RESULT;
            break;
        
        case PatrolState::WAITING_RESULT:
            // 什么都不做，等待goal_result_callback被触发
            break;

        case PatrolState::GOING_HOME:
            send_goal(home_pose_);
            state_ = PatrolState::WAITING_RESULT;
            break;

        case PatrolState::IDLE:
            RCLCPP_INFO(this->get_logger(), "All tasks done. Node is now idle.");
            timer_->cancel(); // 停止定时器，节点进入空闲状态
            break;
        }
    }

    void send_goal(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal to: x=%.2f, y=%.2f", target_pose.pose.position.x, target_pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&PatrolDriver::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&PatrolDriver::goal_result_callback, this, std::placeholders::_1);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void goal_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                // 根据当前状态决定下一步
                if (state_ == PatrolState::WAITING_RESULT) { // 刚完成一个巡逻点
                    current_point_ = (current_point_ + 1) % 2;
                    patrol_count_++;
                    RCLCPP_INFO(this->get_logger(), "Patrol count: %d", patrol_count_);

                    if (patrol_count_ >= 4) {
                        RCLCPP_INFO(this->get_logger(), "Patrol complete. Going home...");
                        state_ = PatrolState::GOING_HOME;
                    } else {
                        state_ = PatrolState::SENDING_GOAL;
                    }
                } else if (state_ == PatrolState::GOING_HOME) { // 刚回到家
                    RCLCPP_INFO(this->get_logger(), "Arrived home.");
                    state_ = PatrolState::IDLE;
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                // 可以选择重试或停止
                state_ = PatrolState::IDLE;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                state_ = PatrolState::IDLE;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                state_ = PatrolState::IDLE;
                break;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    // --- 成员变量 ---
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PatrolState state_;
    int current_point_;
    int patrol_count_;
    
    geometry_msgs::msg::PoseStamped points_[2];
    geometry_msgs::msg::PoseStamped home_pose_;
    geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
