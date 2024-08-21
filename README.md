# bt_navigator_live

An implementation of the default Nav2 behaviour tree navigator that supports Groot Live Monitoring.

check behavior_tree_engine
check bt_navigator

to write a navigator: 

1. see if you can put a `BT::PublisherZMQ` in a derived class of `BehaviorTreeNavigator`.
2. If that works then, write a new action definition removing the option for choosing the tree.
3. create Navigator similar to `NavigateToPoseNavigator` as plugin. thats it.

getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) (source)
goalReceived(ActionT::Goal::ConstSharedPtr goal) (source)
onLoop() (source)
onPreempt(ActionT::Goal::ConstSharedPtr goal) (source)
goalCompleted(typename ActionT::Result::SharedPtr result, const nav2_behavior_tree::BtStatus final_bt_status) (source)
configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node, std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) (source)
cleanup() (source)
getName() (source)