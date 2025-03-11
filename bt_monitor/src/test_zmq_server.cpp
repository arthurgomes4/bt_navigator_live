#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace BT;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a simple Behavior Tree
    BehaviorTreeFactory factory;
    factory.registerNodeType<AlwaysSuccess>("AlwaysSuccess");
    factory.registerNodeType<Sequence>("Sequence");

    auto tree = factory.createTreeFromText(R"(
        <root main_tree_to_execute='MainTree'>
            <BehaviorTree ID='MainTree'>
                <Sequence name='root_sequence'>
                    <AlwaysSuccess name='action1'/>
                    <AlwaysSuccess name='action2'/>
                </Sequence>
            </BehaviorTree>
        </root>
    )");

    // Create the ZMQ publisher
    PublisherZMQ publisher(tree, 25, 1666, 1667);

    // Spin the ROS node
    rclcpp::spin_some(rclcpp::Node::make_shared("bt_zmq_publisher_node"));

    rclcpp::shutdown();
    return 0;
}