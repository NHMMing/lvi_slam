# launch/lvi_slam.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lvi_slam')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg, 'config', 'livox_mid360.yaml']),
        description='算法参数文件')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    slam_node = Node(
        package='lvi_slam',
        executable='lvi_slam_node',
        name='lvi_slam',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
        # 实时性关键：状态估计线程被其他进程抢占会直接导致丢帧。
        # 生产环境建议配合 chrt 设置实时优先级。
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([pkg, 'config', 'lvi_slam.rviz'])],
    )

    return LaunchDescription([config_arg, rviz_arg, slam_node, rviz_node])
