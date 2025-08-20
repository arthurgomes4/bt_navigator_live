#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "bt_monitor/ros2_topic_logger.hpp"
#include <chrono>
#include <thread>

using namespace BT;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_ros2_logger_example");
    node->declare_parameter("use_groot_monitor", true);

    // Create a simple Behavior Tree
    BehaviorTreeFactory factory;
    
    // We don't need to register these nodes as they are built-in
    // The XML parser will handle them automatically
    
    auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SetBlackboard output_key="message" value="Hello World" name="set_message"/>
            <Delay delay_msec="1000">
                <AlwaysSuccess name="action_1"/>
            </Delay>
            <Delay delay_msec="1000">
                <AlwaysSuccess name="action_2"/>
            </Delay>
            <Delay delay_msec="1000">
                <AlwaysSuccess name="action_3"/>
            </Delay>
            <Delay delay_msec="1000">
                <AlwaysSuccess name="action_4"/>
            </Delay>
            <Delay delay_msec="1000">
                <AlwaysSuccess name="action_5"/>
            </Delay>
        </Sequence>
    </BehaviorTree>

    <!-- Tree Parameters -->
    <TreeNodesModel>
        <Action ID="AlwaysSuccess"/>
        <Action ID="AlwaysFailure"/>
        <Action ID="SetBlackboard">
            <input_port name="value"/>
            <output_port name="output_key"/>
        </Action>
        <DecoratorNode ID="Delay">
            <input_port name="delay_msec">Delay in milliseconds</input_port>
        </DecoratorNode>
    </TreeNodesModel>
    </root>
    )");

    // Create the ROS2 topic logger
    bt_monitor::ROS2TopicLogger logger(node, tree);
    
    RCLCPP_INFO(node->get_logger(), "BT ROS2 Logger Example: Starting execution");
    RCLCPP_INFO(node->get_logger(), "Publishing to topics: /full_bt and /bt_updates");
    RCLCPP_INFO(node->get_logger(), "You can monitor this tree with Groot by connecting to these topics");

    // Create a separate thread for spinning the node
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    // Execute the behavior tree
    int count = 0;
    while (rclcpp::ok() && count < 10) {
        auto status = tree.tickRoot();
        RCLCPP_INFO(node->get_logger(), "Tree tick %d returned status: %d", 
                   count, static_cast<int>(status));
        
        if (status != NodeStatus::RUNNING) {
            // We can't directly set the status, so we'll reset the tree
            tree.haltTree();
            count++;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "BT ROS2 Logger Example: Execution completed");
    
    // Keep the node running to allow Groot to connect
    bool use_groot_monitor = node->get_parameter("use_groot_monitor").as_bool();
    if (use_groot_monitor) {
        RCLCPP_INFO(node->get_logger(), "Keeping node alive for Groot monitoring. Press Ctrl+C to exit.");
        while (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    
    return 0;
} 