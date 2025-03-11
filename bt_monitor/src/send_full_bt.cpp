#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h"

class SendFullBT : public rclcpp::Node
{
public:
    SendFullBT() : Node("bt_monitor_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/full_bt", 10);

        BT::BehaviorTreeFactory factory;

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
        // Publish the serialized Behavior Tree
        auto message = std_msgs::msg::ByteMultiArray();

        std::vector<uint8_t> tree_buffer_;
        
        flatbuffers::FlatBufferBuilder builder(1024);
        CreateFlatbuffersBehaviorTree(builder, tree);

        tree_buffer_.resize(builder.GetSize());
        memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

        message.data.resize(tree_buffer_.size());
        memcpy(message.data.data(), tree_buffer_.data(), tree_buffer_.size());

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published serialized BT message");
    }

private:
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendFullBT>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}