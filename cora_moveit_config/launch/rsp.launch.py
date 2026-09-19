# TODO: #2 MoveItConfigsBuilder("cora", package_name="cora_moveit_config") is
# hardcoded in every launch file here. Drive robot_name/package_name from a
# launch arg defaulted off the generated robot_layout.yaml
# (C-O-R-A/configurator#2).
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("cora", package_name="cora_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
