from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the test_ros2_topic_logger node
        Node(
            package='bt_monitor',
            executable='test_ros2_topic_logger',
            name='bt_ros2_logger_example',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_groot': True}
            ]
        ),
        
        # Launch Groot with a delay to ensure the test node is running first
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 2 && ros2 run bt_monitor Groot'],
            output='screen'
        )
    ]) 