from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='WebSocket port for rosbridge_server',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'address': '0.0.0.0',
                'port': port,
            }],
        ),
        Node(
            package='px4_gui_ctrl',
            executable='web_command_relay',
            name='ugv_web_command_relay',
            output='screen',
            parameters=[{
                'web_command_topic': '/web_ui/flight_command',
                'mission_cmd_topic': '/flight_control/mission_cmd',
                'status_topic': '/web_ui/flight_command/status',
            }],
        ),
    ])
