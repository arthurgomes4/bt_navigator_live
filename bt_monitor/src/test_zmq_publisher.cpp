#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace BT;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a simple Behavior Tree
    BehaviorTreeFactory factory;
    // factory.registerNodeType<AlwaysSuccess>("AlwaysSuccess");
    // factory.registerNodeType<Sequence>("Sequence");

    auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
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
        <DecoratorNode ID="Delay">
            <input_port name="delay_msec">Delay in milliseconds</input_port>
        </DecoratorNode>
    </TreeNodesModel>
    </root>
    )");

    // Create the ZMQ publisher
    PublisherZMQ publisher(tree, 25, 1666, 1667);

    // Execute the behavior tree
    while (rclcpp::ok() && tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Spin the ROS node
    rclcpp::spin(rclcpp::Node::make_shared("bt_zmq_publisher_node"));

    rclcpp::shutdown();
    return 0;
}