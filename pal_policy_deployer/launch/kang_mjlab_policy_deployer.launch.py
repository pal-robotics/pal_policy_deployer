from dataclasses import dataclass

import os

from kangaroo_description.launch_arguments import KangarooArgs
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_pal.robot_arguments import CommonArgs
from ament_index_python.packages import get_package_share_directory

from launch_pal.arg_utils import LaunchArgumentsBase


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    # ["True", "False"]
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time

    # [True, False]
    has_pelvis: DeclareLaunchArgument = KangarooArgs.has_pelvis

    # ["no-arm", "4dof", "5dof", "7dof"]
    arm_type: DeclareLaunchArgument = KangarooArgs.arm_type

    activate_impedance_controller: DeclareLaunchArgument = DeclareLaunchArgument(
        name='activate_impedance_controller',
        default_value='True',
        choices=['True', 'False'],
        description='Argument to choose whether to activate the impedance '
                    'controller or leave it configured and in inactive state')


# Maps a (has_pelvis, arm_type) combination to the launch file that deploys the
# matching robot configuration. Extend this table as more configurations are supported.
_ROBOT_CONFIG_LAUNCH_FILES = {
    ("True", "no-arm"): "kang_lowerbody_mjlab_policy_deployer.launch.py",
    ("True", "4dof"): "kang_full_body_4dof_mjlab_policy_deployer.launch.py",
}


def _dispatch_robot_config_launch(context, *args, **kwargs):
    """Include the policy-deployer launch file matching has_pelvis/arm_type.

    Runs as an OpaqueFunction so the (CLI-overridable) ``has_pelvis``/``arm_type``
    launch arguments can be resolved at launch time and used to pick which robot
    configuration's launch file to include. Every other launch argument (e.g.
    ``use_sim_time``, ``activate_impedance_controller``, ``task``, ``model``) is
    forwarded automatically: launch configurations already set on this context
    (from the command line or a parent include) remain visible to the included
    launch file when it declares its own matching arguments, so no explicit
    ``launch_arguments`` mapping is needed here.

    :raises RuntimeError: if the has_pelvis/arm_type combination is not supported.
    """
    has_pelvis = LaunchConfiguration("has_pelvis").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)

    launch_file_name = _ROBOT_CONFIG_LAUNCH_FILES.get((has_pelvis, arm_type))
    if launch_file_name is None:
        raise RuntimeError(
            f"[pal_policy_deployer] Unsupported combination: has_pelvis='{has_pelvis}', "
            f"arm_type='{arm_type}'. Supported combinations (has_pelvis, arm_type) are: "
            f"{sorted(_ROBOT_CONFIG_LAUNCH_FILES.keys())}."
        )

    pkg_share_folder = get_package_share_directory("pal_policy_deployer")
    launch_file_path = os.path.join(pkg_share_folder, "launch", launch_file_name)

    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))]


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    launch_description.add_action(OpaqueFunction(function=_dispatch_robot_config_launch))


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
