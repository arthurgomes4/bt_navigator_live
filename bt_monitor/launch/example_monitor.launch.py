from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Example launch file demonstrating ROS2TopicLogger with a simple test BT.
    
    This shows how to use bt_monitor's general-purpose logging with any BT application.
    """
    
    # Test BT node with integrated logger
    test_logger_node = Node(
        package='bt_monitor',
        executable='test_ros2_topic_logger',
        name='bt_ros2_logger_example',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_groot_monitor': True
        }]
    )
    
    # BT Monitor GUI
    bt_monitor_gui = Node(
        package='bt_monitor',
        executable='bt_monitor',
        name='bt_monitor_gui',
        output='screen'
    )

    return LaunchDescription([
        test_logger_node,
        bt_monitor_gui,
    ])
