#include <array>
#include <future>
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h"

namespace BT
{
class PublisherROS2 : public StatusChangeLogger
{
public:
  PublisherROS2(
    rclcpp::Node::SharedPtr node,
    const BT::Tree& tree, 
    unsigned max_msg_per_second = 25);

  virtual ~PublisherROS2();

  void setBT(const BT::Tree& tree);

private:
  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;

  virtual void flush() override;

  void createStatusBuffer();
  void publishFullTree();
  void publishUpdates();

  const BT::Tree* tree_;
  std::vector<uint8_t> tree_buffer_;
  std::vector<uint8_t> status_buffer_;
  std::vector<SerializedTransition> transition_buffer_;
  std::chrono::microseconds min_time_between_msgs_;

  std::atomic_bool active_server_;
  std::mutex mutex_;
  std::atomic_bool send_pending_;
  std::condition_variable send_condition_variable_;
  std::future<void> send_future_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr full_bt_publisher_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr updates_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

PublisherROS2::PublisherROS2(
    rclcpp::Node::SharedPtr node,
    const BT::Tree& tree, 
    unsigned max_msg_per_second) :
  StatusChangeLogger(tree.rootNode()),
  tree_(&tree),
  min_time_between_msgs_(std::chrono::microseconds(1000 * 1000) / max_msg_per_second),
  send_pending_(false),
  active_server_(true),
  node_(node)
{
  // Create publishers
  full_bt_publisher_ = node_->create_publisher<std_msgs::msg::ByteMultiArray>("/full_bt", 10);
  updates_publisher_ = node_->create_publisher<std_msgs::msg::ByteMultiArray>("/bt_updates", 10);
  
  // Create the timer
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000 / max_msg_per_second),
    std::bind(&PublisherROS2::publishUpdates, this));

  flatbuffers::FlatBufferBuilder builder(1024);
  CreateFlatbuffersBehaviorTree(builder, tree);

  tree_buffer_.resize(builder.GetSize());
  memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

  publishFullTree();
  createStatusBuffer();
}

PublisherROS2::~PublisherROS2()
{
  active_server_ = false;
  if (send_pending_)
  {
    send_condition_variable_.notify_all();
    send_future_.get();
  }
  flush();
  
  // Clean up publishers and timer
  full_bt_publisher_.reset();
  updates_publisher_.reset();
  timer_.reset();
}

void PublisherROS2::setBT(const BT::Tree& tree)
{
  std::unique_lock<std::mutex> lock(mutex_);
  tree_ = &tree;

  flatbuffers::FlatBufferBuilder builder(1024);
  CreateFlatbuffersBehaviorTree(builder, tree);

  tree_buffer_.resize(builder.GetSize());
  memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

  publishFullTree();
  createStatusBuffer();
}

void PublisherROS2::createStatusBuffer()
{
  status_buffer_.clear();
  applyRecursiveVisitor(tree_->rootNode(), [this](TreeNode* node) {
    size_t index = status_buffer_.size();
    status_buffer_.resize(index + 3);
    flatbuffers::WriteScalar<uint16_t>(&status_buffer_[index], node->UID());
    flatbuffers::WriteScalar<int8_t>(
        &status_buffer_[index + 2],
        static_cast<int8_t>(convertToFlatbuffers(node->status())));
  });
}

void PublisherROS2::callback(Duration timestamp, const TreeNode& node,
                            NodeStatus prev_status, NodeStatus status)
{
  SerializedTransition transition =
      SerializeTransition(node.UID(), timestamp, prev_status, status);
  {
    std::unique_lock<std::mutex> lock(mutex_);
    transition_buffer_.push_back(transition);
  }

  if (!send_pending_.exchange(true))
  {
    send_future_ = std::async(std::launch::async, [this]() {
      std::unique_lock<std::mutex> lock(mutex_);
      const bool is_server_inactive = send_condition_variable_.wait_for(
          lock, min_time_between_msgs_, [this]() { return !active_server_; });
      lock.unlock();
      if (!is_server_inactive)
      {
        flush();
      }
    });
  }
}

void PublisherROS2::flush()
{
  auto message = std_msgs::msg::ByteMultiArray();
  {
    std::unique_lock<std::mutex> lock(mutex_);

    const size_t msg_size = status_buffer_.size() + 8 + (transition_buffer_.size() * 12);

    message.data.resize(msg_size);
    uint8_t* data_ptr = message.data.data();

    // first 4 bytes are the side of the header
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(status_buffer_.size()));
    data_ptr += sizeof(uint32_t);
    // copy the header part
    memcpy(data_ptr, status_buffer_.data(), status_buffer_.size());
    data_ptr += status_buffer_.size();

    // first 4 bytes are the side of the transition buffer
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(transition_buffer_.size()));
    data_ptr += sizeof(uint32_t);

    for (auto& transition : transition_buffer_)
    {
      memcpy(data_ptr, transition.data(), transition.size());
      data_ptr += transition.size();
    }
    
    std::cout << "[PublisherROS2] Preparing update message: size=" << msg_size 
              << " header_size=" << status_buffer_.size() 
              << " transitions=" << transition_buffer_.size() << std::endl;
              
    transition_buffer_.clear();
    createStatusBuffer();
  }
  try
  {
    updates_publisher_->publish(message);
    std::cout << "[PublisherROS2] Published update message with size " << message.data.size() << std::endl;
  }
  catch (const std::exception& err)
  {
    std::cerr << "[PublisherROS2] Publisher just died. Exception " << err.what() << std::endl;
  }

  send_pending_ = false;
}

void PublisherROS2::publishFullTree()
{
  auto message = std_msgs::msg::ByteMultiArray();
  message.data.resize(tree_buffer_.size());
  memcpy(message.data.data(), tree_buffer_.data(), tree_buffer_.size());
  full_bt_publisher_->publish(message);
  std::cout << "Published full BT message" << std::endl;
}

void PublisherROS2::publishUpdates()
{
  if (send_pending_)
  {
    std::cout << "[PublisherROS2] Timer triggered update publish" << std::endl;
    flush();
  }
}

}   // namespace BT

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_publisher_node");
  BT::BehaviorTreeFactory factory;

  BT::Tree tree = factory.createTreeFromText(R"(
  <root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
      <Sequence name="root_sequence">
          <Delay delay_msec="5000">
              <AlwaysSuccess name="action_1"/>
          </Delay>
          <Delay delay_msec="5000">
              <AlwaysSuccess name="action_2"/>
          </Delay>
          <Delay delay_msec="5000">
              <AlwaysSuccess name="action_3"/>
          </Delay>
          <Delay delay_msec="5000">
              <AlwaysSuccess name="action_4"/>
          </Delay>
          <Delay delay_msec="5000">
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

  // Create the publisher with node
  BT::PublisherROS2 publisher(node, tree);

  while (rclcpp::ok() && tree.tickRoot() == BT::NodeStatus::RUNNING)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}