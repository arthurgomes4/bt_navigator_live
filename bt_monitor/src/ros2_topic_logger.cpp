#include "bt_monitor/ros2_topic_logger.hpp"
#include <iostream>

namespace bt_monitor
{

ROS2TopicLogger::ROS2TopicLogger(
    rclcpp::Node::SharedPtr node,
    const BT::Tree& tree,
    const std::string& full_tree_topic,
    const std::string& updates_topic,
    unsigned max_msg_per_second) :
  BT::StatusChangeLogger(tree.rootNode()),
  tree_(&tree),
  min_time_between_msgs_(std::chrono::microseconds(1000 * 1000) / max_msg_per_second),
  send_pending_(false),
  active_server_(true),
  node_(node)
{
  // Create publishers with appropriate QoS settings
  // Full BT topic uses transient_local to act as a latched topic
  auto full_bt_qos = rclcpp::QoS(10).transient_local();
  full_bt_publisher_ = node_->create_publisher<std_msgs::msg::ByteMultiArray>(full_tree_topic, full_bt_qos);
  
  // Updates topic uses default QoS for real-time updates
  updates_publisher_ = node_->create_publisher<std_msgs::msg::ByteMultiArray>(updates_topic, 10);
  
  // Create the timer
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000 / max_msg_per_second),
    std::bind(&ROS2TopicLogger::publishUpdates, this));

  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Creating flatbuffer for behavior tree");
  
  flatbuffers::FlatBufferBuilder builder(1024);
  BT::CreateFlatbuffersBehaviorTree(builder, tree);

  tree_buffer_.resize(builder.GetSize());
  memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Tree buffer size: %zu bytes", tree_buffer_.size());
  
  // Publish the tree structure once - the latched topic will retain it for late subscribers
  publishFullTree();
  
  createStatusBuffer();
  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Initialized successfully");
}

ROS2TopicLogger::~ROS2TopicLogger()
{
  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Shutting down");
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

void ROS2TopicLogger::setBT(const BT::Tree& tree)
{
  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Setting new behavior tree");
  std::unique_lock<std::mutex> lock(mutex_);
  tree_ = &tree;

  flatbuffers::FlatBufferBuilder builder(1024);
  BT::CreateFlatbuffersBehaviorTree(builder, tree);

  tree_buffer_.resize(builder.GetSize());
  memcpy(tree_buffer_.data(), builder.GetBufferPointer(), builder.GetSize());

  // Publish the tree structure once - the latched topic will retain it for late subscribers
  publishFullTree();
  
  createStatusBuffer();
}

void ROS2TopicLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, 
                              BT::NodeStatus prev_status, BT::NodeStatus status)
{
  using namespace BT;

  if (!active_server_ || status == prev_status)
  {
    return;
  }

  // Use SerializeTransition to create a SerializedTransition
  SerializedTransition transition = SerializeTransition(node.UID(), timestamp, prev_status, status);

  {
    std::unique_lock<std::mutex> lock(mutex_);
    transition_buffer_.push_back(transition);
  }

  if (!send_pending_)
  {
    send_pending_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "ROS2TopicLogger: Status change detected for node [%s] UID=%d: %d -> %d", 
                node.name().c_str(), node.UID(), 
                static_cast<int>(prev_status), static_cast<int>(status));
  }
}

void ROS2TopicLogger::flush()
{
  auto message = std_msgs::msg::ByteMultiArray();
  {
    std::unique_lock<std::mutex> lock(mutex_);

    const size_t msg_size = status_buffer_.size() + 8 + (transition_buffer_.size() * 12);

    message.data.resize(msg_size);
    uint8_t* data_ptr = message.data.data();

    // first 4 bytes are the size of the header
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(status_buffer_.size()));
    data_ptr += sizeof(uint32_t);
    // copy the header part
    memcpy(data_ptr, status_buffer_.data(), status_buffer_.size());
    data_ptr += status_buffer_.size();

    // next 4 bytes are the size of the transition buffer
    flatbuffers::WriteScalar<uint32_t>(data_ptr,
                                       static_cast<uint32_t>(transition_buffer_.size()));
    data_ptr += sizeof(uint32_t);

    for (auto& transition : transition_buffer_)
    {
      memcpy(data_ptr, transition.data(), transition.size());
      data_ptr += transition.size();
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "ROS2TopicLogger: Preparing update message: size=%zu header_size=%zu transitions=%zu",
                msg_size, status_buffer_.size(), transition_buffer_.size());
              
    transition_buffer_.clear();
    createStatusBuffer();
  }
  try
  {
    updates_publisher_->publish(message);
    RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Published update message with size %zu", message.data.size());
  }
  catch (const std::exception& err)
  {
    RCLCPP_ERROR(node_->get_logger(), "ROS2TopicLogger: Publisher error: %s", err.what());
  }

  send_pending_ = false;
}

void ROS2TopicLogger::createStatusBuffer()
{
  status_buffer_.clear();
  BT::applyRecursiveVisitor(tree_->rootNode(), [this](BT::TreeNode* node) {
    size_t index = status_buffer_.size();
    status_buffer_.resize(index + 3);
    flatbuffers::WriteScalar<uint16_t>(&status_buffer_[index], node->UID());
    flatbuffers::WriteScalar<int8_t>(
        &status_buffer_[index + 2],
        static_cast<int8_t>(BT::convertToFlatbuffers(node->status())));
    
    RCLCPP_DEBUG(node_->get_logger(), "ROS2TopicLogger: Added node to status buffer: [%s] UID=%d status=%d", 
                node->name().c_str(), node->UID(), static_cast<int>(node->status()));
  });
  
  RCLCPP_DEBUG(node_->get_logger(), "ROS2TopicLogger: Status buffer created with size: %zu", status_buffer_.size());
}

void ROS2TopicLogger::publishFullTree()
{
  auto message = std_msgs::msg::ByteMultiArray();
  message.data.resize(tree_buffer_.size());
  memcpy(message.data.data(), tree_buffer_.data(), tree_buffer_.size());
  full_bt_publisher_->publish(message);
  RCLCPP_INFO(node_->get_logger(), "ROS2TopicLogger: Published full BT message with size %zu", tree_buffer_.size());
}

void ROS2TopicLogger::publishUpdates()
{
  if (send_pending_)
  {
    RCLCPP_DEBUG(node_->get_logger(), "ROS2TopicLogger: Timer triggered update publish");
    flush();
  }
}

}  // namespace bt_monitor 