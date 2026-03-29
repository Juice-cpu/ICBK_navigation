#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>

class SerpentinePatrolNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    SerpentinePatrolNode() : Node("serpentine_patrol_node")
    {
        RCLCPP_INFO(this->get_logger(), "Serpentine Patrol Node started.");

        // 逐点导航，行为与 RViz 单点 goal 一致
        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        initializeWaypoints();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerpentinePatrolNode::wait_and_send_goal, this));
    }

private:
    geometry_msgs::msg::PoseStamped makePose(double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0; // 初始朝向，后续按路径方向更新
        return pose;
    }

    void applyHeadingAlongPath()
    {
        if (waypoints_.size() < 2) {
            return;
        }
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            size_t next = (i + 1 < waypoints_.size()) ? i + 1 : i;
            double dx = waypoints_[next].pose.position.x - waypoints_[i].pose.position.x;
            double dy = waypoints_[next].pose.position.y - waypoints_[i].pose.position.y;
            double yaw = std::atan2(dy, dx);
            double half_yaw = yaw * 0.5;
            waypoints_[i].pose.orientation.w = std::cos(half_yaw);
            waypoints_[i].pose.orientation.z = std::sin(half_yaw); // 平面运动仅 z 分量
        }
    }

    void initializeWaypoints()
    {
        // 路径点来自你在 RViz 点击的坐标，按顺序逐点导航
        waypoints_.clear();

        const std::vector<std::pair<double, double>> pts = {
            {1.59, -0.78},
            {2.64,  0.09},
            {1.82,  1.57},
            {3.27,  1.89},
            {3.17,  0.44},
            {3.45, -0.96},
            {4.46, -0.15},
            {3.62,  1.03},
            {4.78,  1.93},
            {5.26,  0.46}
        };

        for (const auto & p : pts) {
            waypoints_.push_back(makePose(p.first, p.second));
        }

        applyHeadingAlongPath();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f)",
                i, waypoints_[i].pose.position.x, waypoints_[i].pose.position.y);
        }

        RCLCPP_INFO(this->get_logger(), "Initialized %zu waypoints for serpentine path.", waypoints_.size());
        current_index_ = 0;
    }

    void wait_and_send_goal()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
            return;
        }
        timer_->cancel();
        start_time_ = std::chrono::steady_clock::now();
        auto start_timepoint = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(start_timepoint);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        RCLCPP_INFO(this->get_logger(), "=== Navigation Started at: %s ===", ss.str().c_str());

        send_goal();
    }

    void send_goal()
    {
        if (current_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[current_index_];

        RCLCPP_INFO(this->get_logger(), "Sending waypoint %zu / %zu -> (%.2f, %.2f)",
            current_index_, waypoints_.size() - 1,
            goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SerpentinePatrolNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SerpentinePatrolNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&SerpentinePatrolNode::result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, navigating...");
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/)
    {
        // 可在此添加反馈日志
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);

        auto end_timepoint = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(end_timepoint);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached at %s, time %.3f s",
                    current_index_, ss.str().c_str(), elapsed.count() / 1000.0);
                ++current_index_;
                start_time_ = std::chrono::steady_clock::now();
                send_goal();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Goal was aborted. Time elapsed: %.3f seconds",
                    elapsed.count() / 1000.0);
                rclcpp::shutdown();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled. Time elapsed: %.3f seconds",
                    elapsed.count() / 1000.0);
                rclcpp::shutdown();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code. Time elapsed: %.3f seconds",
                    elapsed.count() / 1000.0);
                rclcpp::shutdown();
                break;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_index_{0};
    std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerpentinePatrolNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
