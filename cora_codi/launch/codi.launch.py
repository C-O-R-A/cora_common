import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    codi_server_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('cora_codi'),
            'config',
            'server_params.yaml'
        ),
    )

    codi_node = Node(
        package="cora_codi",
        executable="codi_node",
        name="codi_node",
        output="both",
        parameters=[{
            "config_file": LaunchConfiguration('config_file')
        }],
    )

    return LaunchDescription([
            codi_server_config_arg,
            codi_node,
        ])
