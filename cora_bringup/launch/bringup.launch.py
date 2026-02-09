import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnExecutionComplete,
)
import os


def generate_launch_description():

    ##############
    # File Paths #
    ##############

    # CoDi server config file path
    config_file = os.path.join(
        get_package_share_directory("cora_codi"), "config", "server_params.yaml"
    )

    ####################
    # Launch arguments #
    ####################

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_codi",
            default_value="true",
            description="Whether to use CoDi for control",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="Fake",
            description="Choose hardware type: Fake or Real",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_package",
            default_value="None",
            description="Name of gripper description package",
        )
    )

    ########################
    # Launch Configuration #
    ########################

    codi_launch_condition = IfCondition(LaunchConfiguration("use_codi"))
    gripper_package = LaunchConfiguration("gripper_package")
    hardware_type = LaunchConfiguration("hardware_type")

    #########
    # Nodes #
    #########

    # Include CoDi node
    codi_node = Node(
        package="cora_codi",
        executable="codi_node",
        name="codi_node",
        output="both",
        parameters=[{"config_file": config_file}],
        condition=codi_launch_condition,
    )

    ########################
    # Launch file Includes #
    ########################

    # Include MoveIt and Mover node launch files,
    # these will also launch RViz, tf2, controllers, etc
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cora_moveit"), "launch", "move.launch.py"
            )
        ),
        launch_arguments={
            "use_servo": "true",
            "use_moveitpy": "true",
            "gripper_package": gripper_package,
            "hardware_type": hardware_type,
        }.items(),
    )

    # Only start CoDi after MoveIt launch finishes initialization
    codi_after_moveit = TimerAction(period=30.0, actions=[codi_node])

    return LaunchDescription(
        declared_arguments
        + [
            moveit_launch,
            codi_after_moveit,
        ]
    )


# ros2 launch cora_bringup bringup.launch.py use_codi:=true hardware:="Fake" gripper_package:='cora_gripper_1_description'