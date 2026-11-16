"""Display the 20-joint model in ROS 2 / RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('tsensei_hand_description'))
    description = (share / 'urdf/tsensei_hand_left_ros.urdf').read_text(encoding='utf-8')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher', condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/display.rviz')], condition=IfCondition(rviz)),
    ])
