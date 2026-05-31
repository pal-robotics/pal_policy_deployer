from dataclasses import dataclass

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description

from launch_pal.arg_utils import LaunchArgumentsBase


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )

    task: DeclareLaunchArgument = DeclareLaunchArgument(
        name="task",
        default_value="loco",
        description="Task name for the policy to deploy.",
    )

    model: DeclareLaunchArgument = DeclareLaunchArgument(
        name="model",
        default_value="v260",
        description="Model version of the policy to deploy.",
    )


def resolve_policy_model(pkg_share_folder: str, task: str, model: str) -> str:
    """Validate that the policy model exists and return its directory.

    Mirrors how kang_mjlab_node locates models at runtime (see
    kang_mjlab_node.cpp::loadPolicy): the model lives in
    ``<share>/models/<task>/<task>_<model>_mjlab/`` and that directory must
    contain at least one ``.onnx`` file.

    :param pkg_share_folder: share directory of the pal_policy_deployer package.
    :param task: policy task name (e.g. ``loco``).
    :param model: policy model version (e.g. ``v260``).
    :returns: absolute path to the validated model directory.
    :raises FileNotFoundError: if the directory is missing or has no ``.onnx`` file.
    """
    model_dir = os.path.join(pkg_share_folder, "models", task, f"{task}_{model}_mjlab")
    if not os.path.isdir(model_dir):
        raise FileNotFoundError(
            f"[pal_policy_deployer] Policy model directory not found: '{model_dir}'. "
            f"Check the 'task' (='{task}') and 'model' (='{model}') launch arguments."
        )
    onnx_files = [f for f in os.listdir(model_dir) if f.endswith(".onnx")]
    if not onnx_files:
        raise FileNotFoundError(
            f"[pal_policy_deployer] No .onnx model file found in '{model_dir}' "
            f"for task='{task}', model='{model}'."
        )
    return model_dir


def _check_model_and_create_actions(context, *args, **kwargs):
    """Verify the policy model exists, then create the node and controller actions.

    Runs as an OpaqueFunction so the (CLI-overridable) ``task``/``model`` launch
    arguments can be resolved at launch time. If the model is missing,
    ``resolve_policy_model`` raises and the launch aborts with a non-zero exit code
    before the node or controller are started (the actions below are only returned
    on the success path).
    """
    pkg_share_folder = get_package_share_directory("pal_policy_deployer")
    task = LaunchConfiguration("task").perform(context)
    model = LaunchConfiguration("model").perform(context)

    # Fail fast (non-zero exit) before launching anything if the model is absent.
    resolve_policy_model(pkg_share_folder, task, model)

    kang_mjlab_node = Node(
        package="pal_policy_inferer",
        executable="kang_mjlab_node",
        name="pal_policy_deployer",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "policies": ["loco"],
            "loco.task": LaunchConfiguration("task"),
            "loco.model": LaunchConfiguration("model"),
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
                "config", "kangaroo_rl_controller.yaml"),
                extra_spawner_args=["--unload-on-kill"])
         ],
        forwarding=False,
    )

    # If the policy node exits for any reason (crash, kill, or normal exit), tear down the
    # whole launch. Shutting down terminates the controller spawner too, and because it was
    # spawned with '--unload-on-kill' the controller is unloaded rather than left running
    # without a policy driving it.
    shutdown_on_node_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=kang_mjlab_node,
            on_exit=[
                LogInfo(
                    msg="[pal_policy_deployer] policy node exited; shutting down the launch "
                        "(controller will be unloaded)."),
                Shutdown(reason="pal_policy_deployer node exited"),
            ],
        )
    )

    return [kang_mjlab_node, rl_impedance_controller, shutdown_on_node_exit]


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    launch_description.add_action(OpaqueFunction(function=_check_model_and_create_actions))


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
