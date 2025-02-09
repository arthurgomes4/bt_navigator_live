/*
    sample program to test the zmq publisher from behaviortreecppv3.
*/
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

static const char* xml_text_subA = R"(
<root>
    <BehaviorTree ID="SubA">
        <SaySomething message="Executing SubA" />
    </BehaviorTree>
</root>  )";

static const char* xml_text_subB = R"(
<root>
    <BehaviorTree ID="SubB">
        <SaySomething message="Executing SubB" />
    </BehaviorTree>
</root>  )";

class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("message") };
    }
};

BT::NodeStatus SaySomething::tick()
{
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");

    // Register the behavior tree definitions, but do not instantiate them yet.
    // Order is not important.
    factory.registerBehaviorTreeFromText(xml_text_subA);
    factory.registerBehaviorTreeFromText(xml_text_subB);

    //Check that the BTs have been registered correctly
    std::cout << "Registered BehaviorTrees:" << std::endl;
    for (const std::string& bt_name : factory.registeredBehaviorTrees())
    {
        std::cout << " - " << bt_name << std::endl;
    }

    auto tree_A = factory.createTree("SubA");
    auto tree_B = factory.createTree("SubB");

    // Create a ZMQ publisher shared pointer
    std::shared_ptr<BT::PublisherZMQ> publisher_zmq_ptr = std::make_shared<BT::PublisherZMQ>(tree_A);
    // BT::PublisherZMQ publisher_zmq(tree_A);

    tree_A.tickRootWhileRunning();

    // delay for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // release delete the shared pointer
    publisher_zmq_ptr.reset();

    publisher_zmq_ptr = std::make_shared<BT::PublisherZMQ>(tree_B);

    tree_B.tickRootWhileRunning();

    // delay for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    publisher_zmq_ptr.reset();
    
    return 0;
}