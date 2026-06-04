import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. fast_lio_localization 런치 파일 포함
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio_localization'),
                'launch',
                'localization.launch.py'
            )
        ),
        # 커맨드라인에서 넘겨주던 인자(Arguments) 전달
        launch_arguments={
            'pcd_map_topic': 'cloud_pcd',
            'map': '/home/harry/indoor_map_0518.pcd'
        }.items()
    )

    # 2. fastlio_to_px4_bridge 노드 실행
    bridge_node = Node(
        package='odometry_bridge',
        executable='fastlio_to_px4_bridge',
        name='fastlio_to_px4_bridge',
        output='screen'
    )

    # 3. odom_monitor_gui 노드 실행
    gui_node = Node(
        package='odometry_bridge',
        executable='odom_monitor_gui',
        name='odom_monitor_gui',
        output='screen'
    )

    # 정의한 모든 액션을 반환하여 동시 실행
    return LaunchDescription([
        fast_lio_launch,
        bridge_node,
        gui_node
    ])