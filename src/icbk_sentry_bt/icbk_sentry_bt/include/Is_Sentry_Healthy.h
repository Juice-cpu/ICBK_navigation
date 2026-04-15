#ifndef Is_Sentry_healthy_h
#define Is_Sentry_healthy_h

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "referee_receiver.h"
#include "pb_extra_nodes.h"  // For global referee source

class IsSentryHealthy : public BT::ConditionNode {
public:
    IsSentryHealthy(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<SentryRefereeReceiver*>("referee_data") };
    }

    BT::NodeStatus tick() override {
        SentryRefereeReceiver* ref_ptr = nullptr;
        try {
            auto ref = getInput<SentryRefereeReceiver*>("referee_data");
            if (ref) {
                ref_ptr = *ref;
            }
        } catch (const std::exception&) {
            // 如果从 blackboard 读取失败，使用全局后备指针
        }
        
        // 如果 blackboard 读取失败，使用全局后备指针
        if (!ref_ptr && g_referee_source) {
            ref_ptr = g_referee_source;
        }
        
        if (!ref_ptr) {
            throw BT::RuntimeError("missing required input [referee_data]: referee source is not set");
        }

        // 修正这里：getLastestData -> getLatestData
        auto data = ref_ptr->getLatestData(); 
        
        if (data.sentry_hp > 50) {
            return BT::NodeStatus::SUCCESS; //健康
        } else {
            return BT::NodeStatus::FAILURE; //不健康
        }
    }
};

#endif // Is_Sentry_healthy_h
