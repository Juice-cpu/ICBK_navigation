#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "referee_receiver.h"
#include "pb_extra_nodes.h"
#include "patrol_between_points.h"
#include "wait_for_navigation.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <streambuf>
#include <thread>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryRefereeReceiver>();

    // 裁判系统接收节点单独放在线程里 spin，行为树主循环只负责 tick。
    rclcpp::executors::SingleThreadedExecutor main_executor;
    main_executor.add_node(node);
    std::thread main_spin_thread([&main_executor]() { main_executor.spin(); });

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<IsGameStatus>("IsGameStatus");
    factory.registerNodeType<IsStatusOK>("IsStatusOK");
    factory.registerNodeType<LowHpLatched>("LowHpLatched");
    factory.registerNodeType<IsAllyOutpostDestroyed>("IsAllyOutpostDestroyed");
    factory.registerNodeType<PublishTwist>("PublishTwist");
    factory.registerNodeType<PubNav2Goal>("PubNav2Goal");
    factory.registerNodeType<PublishCurrentPoseGoal>("PublishCurrentPoseGoal");
    factory.registerNodeType<IsTargetVisible>("IsTargetVisible");
    factory.registerNodeType<IsEcDecisionGoOut>("IsEcDecisionGoOut");
    factory.registerNodeType<IsEcDecisionGoHome>("IsEcDecisionGoHome");
    factory.registerNodeType<PatrolBetweenPoints>("PatrolBetweenPoints");
    factory.registerNodeType<WaitForNavigation>("WaitForNavigation");

    std::string default_bt_path;
    try {
        default_bt_path = ament_index_cpp::get_package_share_directory("icbk_sentry_bt") +
                          "/behavior_trees/sentry_behavior.xml";
    } catch (const std::exception&) {
        default_bt_path = "src/icbk_sentry_bt/behavior_trees/sentry_behavior.xml";
    }

    node->declare_parameter<std::string>("bt_xml_path", default_bt_path);
    node->declare_parameter<int>("bt_tick_hz", 100);
    const std::string bt_path = node->get_parameter("bt_xml_path").as_string();
    int bt_tick_hz = node->get_parameter("bt_tick_hz").as_int();

    if (bt_tick_hz <= 0) {
        RCLCPP_WARN(node->get_logger(), "Invalid bt_tick_hz=%d, fallback to 100", bt_tick_hz);
        bt_tick_hz = 100;
    }

    std::ifstream xml_file(bt_path);
    if (!xml_file.is_open()) {
        RCLCPP_FATAL(node->get_logger(), "Failed to open behavior tree file: %s", bt_path.c_str());
        return 1;
    }
    std::string xml_text(
        (std::istreambuf_iterator<char>(xml_file)),
        std::istreambuf_iterator<char>());

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", bt_path.c_str());

    // 黑板负责在行为树节点之间共享裁判系统指针和关键目标点。
    auto blackboard = BT::Blackboard::create();
    SentryRefereeReceiver* referee_data = node.get();

    // home_point 供回家逻辑使用。
    geometry_msgs::msg::PoseStamped home_point;
    home_point.header.frame_id = "map";
    home_point.pose.position.x = 0.0;
    home_point.pose.position.y = 0.0;
    home_point.pose.orientation.w = 1.0;

    blackboard->set("referee_data", referee_data);
    blackboard->set("referee_gameStatus", referee_data);
    blackboard->set("referee_robotStatus", referee_data);
    blackboard->set("referee_rfidStatus", referee_data);
    blackboard->set("home_point", home_point);
    blackboard->set("@referee_gameStatus", referee_data);
    blackboard->set("@referee_robotStatus", referee_data);
    blackboard->set("@referee_rfidStatus", referee_data);

    // 保留一份全局裁判系统指针，兼容某些节点黑板取指针失败时的兜底路径。
    g_referee_source = referee_data;

    auto tree = factory.createTreeFromText(xml_text, blackboard);
    tree.rootBlackboard()->set("referee_data", referee_data);
    tree.rootBlackboard()->set("referee_gameStatus", referee_data);
    tree.rootBlackboard()->set("referee_robotStatus", referee_data);
    tree.rootBlackboard()->set("referee_rfidStatus", referee_data);
    tree.rootBlackboard()->set("home_point", home_point);
    tree.rootBlackboard()->set("@referee_gameStatus", referee_data);
    tree.rootBlackboard()->set("@referee_robotStatus", referee_data);
    tree.rootBlackboard()->set("@referee_rfidStatus", referee_data);

    RCLCPP_INFO(node->get_logger(), "Behavior tree loaded successfully");
    RCLCPP_INFO(node->get_logger(), "BT tick frequency: %d Hz", bt_tick_hz);

    rclcpp::Rate rate(bt_tick_hz);
    while (rclcpp::ok()) {
        // 处理行为树辅助节点的回调，例如目标缓存订阅。
        if (g_target_node) {
            rclcpp::spin_some(g_target_node);
        }
        tree.tickRoot();
        rate.sleep();
    }

    // 在 ROS 关闭前释放行为树辅助节点，避免悬挂资源。
    cleanupPbExtraNodeResources();
    PatrolBetweenPoints::cleanupSharedResources();
    WaitForNavigation::cleanupSharedResources();

    rclcpp::shutdown();
    if (main_spin_thread.joinable()) {
        main_spin_thread.join();
    }
    return 0;
}
