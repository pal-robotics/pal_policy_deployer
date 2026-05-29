from dataclasses import dataclass

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description

from launch_pal.arg_utils import LaunchArgumentsBase


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    pkg_share_folder = get_package_share_directory("pal_policy_deployer")

    kang_mjlab_node = Node(
        package="pal_policy_inferer",
        executable="kang_mjlab_node",
        name="pal_policy_deployer",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "policies": ["loco"],
            "loco.task": "loco",
            "loco.model": "v260",
        }],
        remappings=[
            ("/subscriber_controller/desired_state", "/rl_impedance_controller/reference"),
        ],
    )

    rl_impedance_controller = GroupAction(
        [generate_load_controller_launch_description(
            controller_name="rl_impedance_controller",
            controller_params_file=os.path.join(
                pkg_share_folder,
                "config", "kangaroo_rl_controller.yaml"))
         ],
        forwarding=False,
    )

    launch_description.add_action(kang_mjlab_node)
    launch_description.add_action(rl_impedance_controller)


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
