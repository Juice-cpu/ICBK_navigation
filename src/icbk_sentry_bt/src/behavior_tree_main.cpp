#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "referee_receiver.h"
#include "pb_extra_nodes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <streambuf>
#include <thread>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryRefereeReceiver>();
    rclcpp::executors::SingleThreadedExecutor main_executor;
    main_executor.add_node(node);
    std::thread main_spin_thread([&main_executor]() { main_executor.spin(); });

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<IsGameStatus>("IsGameStatus");
    factory.registerNodeType<IsStatusOK>("IsStatusOK");
    factory.registerNodeType<LowHpLatched>("LowHpLatched");
    factory.registerNodeType<PublishTwist>("PublishTwist");
    factory.registerNodeType<PubNav2Goal>("PubNav2Goal");
    factory.registerNodeType<PublishCurrentPoseGoal>("PublishCurrentPoseGoal");
    factory.registerNodeType<IsTargetVisible>("IsTargetVisible");

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
        RCLCPP_FATAL(node->get_logger(), "无法打开行为树文件: %s", bt_path.c_str());
        return 1;
    }
    std::string xml_text((std::istreambuf_iterator<char>(xml_file)),
                         std::istreambuf_iterator<char>());

    RCLCPP_INFO(node->get_logger(), "正在从文件加载行为树: %s", bt_path.c_str());

    // 创建黑板并预置裁判数据指针，防止默认值写入错误类型
    auto blackboard = BT::Blackboard::create();
    SentryRefereeReceiver* referee_data = node.get();
    
    // 使用标准 set() 方法，让编译器自动推断类型
    // 注意：BehaviorTree.CPP 的 blackboard 可能无法正确处理原始指针类型
    // 所以我们在节点代码中使用 try-catch 和全局变量作为后备
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
    
    // 设置全局后备指针（这是主要的数据源，因为 blackboard 可能无法正确处理指针类型）
    g_referee_source = referee_data;

    auto tree = factory.createTreeFromText(xml_text, blackboard);
    // 再次写入根黑板，确保派生黑板也能查到同名指针
    tree.rootBlackboard()->set("referee_data", referee_data);
    tree.rootBlackboard()->set("referee_gameStatus", referee_data);
    tree.rootBlackboard()->set("referee_robotStatus", referee_data);
    tree.rootBlackboard()->set("referee_rfidStatus", referee_data);
    tree.rootBlackboard()->set("home_point", home_point);
    tree.rootBlackboard()->set("@referee_gameStatus", referee_data);
    tree.rootBlackboard()->set("@referee_robotStatus", referee_data);
    tree.rootBlackboard()->set("@referee_rfidStatus", referee_data);

    RCLCPP_INFO(node->get_logger(), "行为树加载成功！");

    // BT tick frequency. Main node callbacks are handled by dedicated executor thread.
    RCLCPP_INFO(node->get_logger(), "BT tick frequency: %d Hz", bt_tick_hz);
    rclcpp::Rate rate(bt_tick_hz);
    while (rclcpp::ok()){
        // Spin shared nodes used inside BT actions/conditions.
        if (g_target_node) {
            rclcpp::spin_some(g_target_node);
        }
        tree.tickRoot();
        rate.sleep();
    }
    // Release helper nodes before shutting down ROS context.
    cleanupPbExtraNodeResources();

    rclcpp::shutdown();
    if (main_spin_thread.joinable()) {
        main_spin_thread.join();
    }
    return 0;
}
