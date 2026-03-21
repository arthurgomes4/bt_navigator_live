from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Example launch file showing how to use monitored_bt_navigator with Groot monitoring.
    
    This demonstrates a standalone BT navigator with monitoring enabled.
    In practice, you would typically launch this as part of your full Nav2 stack.
    """
    
    # Declare launch arguments
    enable_groot_arg = DeclareLaunchArgument(
        'enable_groot_monitoring',
        default_value='true',
        description='Enable Groot live monitoring of behavior trees'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to parameters file for bt_navigator'
    )

    # Monitored BT Navigator node
    monitored_bt_navigator_node = Node(
        package='bt_navigator_live',
        executable='monitored_bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                # Groot monitoring configuration
                'enable_groot_monitoring': LaunchConfiguration('enable_groot_monitoring'),
                'groot_full_bt_topic': '/bt_navigator/full_bt',
                'groot_updates_topic': '/bt_navigator/bt_updates',
                
                # Standard bt_navigator parameters (examples)
                # These would typically come from your params_file
                # 'default_nav_to_pose_bt_xml': '...',
                # 'default_nav_through_poses_bt_xml': '...',
                # 'plugin_lib_names': [...],
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    # BT Monitor GUI (optional - only if you want the visual monitor)
    bt_monitor_gui = Node(
        package='bt_monitor',
        executable='bt_monitor',
        name='bt_monitor_gui',
        output='screen',
        parameters=[{
            # GUI runs as regular ROS2 node, subscribes to the topics
        }]
    )

    return LaunchDescription([
        enable_groot_arg,
        params_file_arg,
        monitored_bt_navigator_node,
        bt_monitor_gui,  # Comment this out if you don't want the GUI
    ])
