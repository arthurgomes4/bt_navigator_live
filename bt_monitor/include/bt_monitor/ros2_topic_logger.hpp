#ifndef BT_MONITOR_ROS2_TOPIC_LOGGER_HPP
#define BT_MONITOR_ROS2_TOPIC_LOGGER_HPP

#include <array>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h"

namespace bt_monitor
{

/**
 * @class ROS2TopicLogger
 * @brief A general-purpose behavior tree logger that publishes to ROS2 topics
 * 
 * This logger is designed to work with any BehaviorTree.CPP v3 behavior tree
 * and publishes serialized tree structure and status updates to ROS2 topics
 * in a format compatible with Groot monitoring tools.
 * 
 * The logger publishes:
 * - Full tree structure (latched) - Published once when tree is created
 * - Status updates - Real-time updates of node state changes
 * 
 * This is a standalone, general-purpose component that can be used independently
 * of any specific application (Nav2, etc.). Simply instantiate it with your tree
 * and it will handle all the monitoring communication.
 * 
 * This logger publishes the full behavior tree structure to one topic and status updates to another.
 * It can be used to monitor the execution of a behavior tree in real-time.
 */
class ROS2TopicLogger : public BT::StatusChangeLogger
{
public:
  /**
   * @brief Constructor for ROS2TopicLogger
   * 
   * @param node ROS2 node to use for publishing
   * @param tree The behavior tree to monitor
   * @param full_tree_topic Topic name for publishing the full tree structure (default: "/full_bt")
   * @param updates_topic Topic name for publishing status updates (default: "/bt_updates")
   * @param max_msg_per_second Maximum number of messages to publish per second (default: 25)
   */
  ROS2TopicLogger(
    rclcpp::Node::SharedPtr node,
    const BT::Tree& tree,
    const std::string& full_tree_topic = "/full_bt",
    const std::string& updates_topic = "/bt_updates",
    unsigned max_msg_per_second = 25);

  /**
   * @brief Destructor
   */
  virtual ~ROS2TopicLogger();

  /**
   * @brief Set a new behavior tree to monitor
   * 
   * @param tree The new behavior tree to monitor
   */
  void setBT(const BT::Tree& tree);

private:
  /**
   * @brief Callback for status changes in the behavior tree
   */
  virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, 
                        BT::NodeStatus prev_status, BT::NodeStatus status) override;

  /**
   * @brief Flush the status buffer and publish updates
   */
  virtual void flush() override;

  /**
   * @brief Create the status buffer for the current tree state
   */
  void createStatusBuffer();
  
  /**
   * @brief Publish the full tree structure
   */
  void publishFullTree();
  
  /**
   * @brief Publish status updates
   */
  void publishUpdates();

  // The behavior tree being monitored
  const BT::Tree* tree_;
  
  // Buffers for serialized data
  std::vector<uint8_t> tree_buffer_;
  std::vector<uint8_t> status_buffer_;
  std::vector<BT::SerializedTransition> transition_buffer_;
  
  // Timing control
  std::chrono::microseconds min_time_between_msgs_;

  // Thread synchronization
  std::atomic_bool active_server_;
  std::mutex mutex_;
  std::atomic_bool send_pending_;
  std::condition_variable send_condition_variable_;
  std::future<void> send_future_;

  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr full_bt_publisher_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr updates_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace bt_monitor

#endif  // BT_MONITOR_ROS2_TOPIC_LOGGER_HPP 