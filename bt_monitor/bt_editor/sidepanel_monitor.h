#ifndef SIDEPANEL_MONITOR_H
#define SIDEPANEL_MONITOR_H

#include <QFrame>

#include "bt_editor_base.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

namespace Ui {
class SidepanelMonitor;
}

class SidepanelMonitor : public QFrame
{
    Q_OBJECT

public:
    /// Timer period in milliseconds.
    static constexpr int _timer_period_ms = 20;
    /// Default timeout to get behavior tree, in milliseconds.
    static constexpr int _load_tree_default_timeout_ms = 1000;
    /// Timeout to get behavior tree during autoconnect, in milliseconds.
    static constexpr int _load_tree_autoconnect_timeout_ms = 10000;

    explicit SidepanelMonitor(QWidget *parent = nullptr,
                              rclcpp::Node::SharedPtr node_ptr = nullptr);
    ~SidepanelMonitor();

    void clear();

    void set_load_tree_timeout_ms(const int timeout_ms)
    {
        _load_tree_timeout_ms = timeout_ms;
    };

public slots:

    void on_Connect();

private slots:

    void on_timer();

signals:
    void loadBehaviorTree(const AbsBehaviorTree& tree, const QString &bt_name );

    void connectionUpdate(bool connected);

    void changeNodeStyle(const QString& bt_name,
                         const std::vector<std::pair<int, NodeStatus>>& node_status);

    void addNewModel(const NodeModel &new_model);

private:
    Ui::SidepanelMonitor *ui;

    QTimer* _timer;

    bool _connected;
    std::string _connection_address_pub;
    std::string _connection_address_req;
    int _msg_count;

    int _load_tree_timeout_ms;  // Timeout to get behavior tree.
    AbsBehaviorTree _loaded_tree;
    std::unordered_map<int, int> _uid_to_index;

    bool getTreeFromServer();

    QWidget *_parent;

    rclcpp::Node::SharedPtr _node_ptr; // Store the node pointer

    // ROS2 subscribers
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr full_bt_subscriber_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr bt_updates_subscriber_;

    // Callback functions
    void fullBtCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
    void btUpdatesCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

};

#endif // SIDEPANEL_MONITOR_H
