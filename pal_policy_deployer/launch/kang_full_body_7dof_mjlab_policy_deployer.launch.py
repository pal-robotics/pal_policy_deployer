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
        description="Task name for the primary (default-active) policy.",
    )

    model: DeclareLaunchArgument = DeclareLaunchArgument(
        name="model",
        description="Model version of the primary (default-active) policy. Required.",
    )

    imitation_task: DeclareLaunchArgument = DeclareLaunchArgument(
        name="imitation_task",
        default_value="imitation",
        description="Task name for the secondary policy switched to on request.",
    )

    imitation_model: DeclareLaunchArgument = DeclareLaunchArgument(
        name="imitation_model",
        description="Model version of the secondary (imitation) policy. Required.",
    )

    imitation_frames: DeclareLaunchArgument = DeclareLaunchArgument(
        name="imitation_frames",
        default_value="300",
        description=(
            "Auto-return frame budget for the imitation policy: after this many "
            "inference frames the node switches back to the primary policy. "
            "Set it to the deployed motion's frame count so the policy returns to "
            "locomotion exactly when the motion ends (e.g. 305 for the v472 jump; "
            "this is the row count of 'joint_pos' in the model's motion.npz). "
            "0 falls back to the node's hardcoded 6000-frame budget."
        ),
    )

    robot_type: DeclareLaunchArgument = DeclareLaunchArgument(
        name="robot_type",
        default_value="full_body_7dof",
        description="Configuration of the robot hardware",
    )

    activate_impedance_controller: DeclareLaunchArgument = DeclareLaunchArgument(
        name='activate_impedance_controller',
        default_value='True',
        choices=['True', 'False'],
        description='Argument to choose whether to activate the impedance '
                    'controller or leave it configured and in inactive state')


def resolve_policy_model(pkg_share_folder: str, task: str, model: str) -> str:
    """Validate that the policy model exists and return its directory.

    The returned directory is passed to kang_mjlab_node as ``<policy>.model_path``.
    The model lives in
    ``<share>/models/full_body_7dof/<task>/<task>_<model>_mjlab/`` and that directory must
    contain at least one ``.onnx`` file.

    :param pkg_share_folder: share directory of the pal_policy_deployer package.
    :param task: policy task name (e.g. ``loco``).
    :param model: policy model version (e.g. ``v260``).
    :returns: absolute path to the validated model directory.
    :raises FileNotFoundError: if the directory is missing or has no ``.onnx`` file.
    """
    model_dir = os.path.join(pkg_share_folder, "models", "full_body_7dof", task, f"{task}_{model}_mjlab")
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
    """Verify the policy models exist, then create the node and controller actions.

    Runs as an OpaqueFunction so the (CLI-overridable) ``task``/``model`` and
    ``imitation_task``/``imitation_model`` launch arguments can be resolved at launch
    time. Two policies are configured: the primary (default-active) policy and the
    imitation policy that the node switches to on request via its ``~/switch_policy``
    action. Both model directories are validated up front; if either is missing,
    ``resolve_policy_model`` raises and the launch aborts with a non-zero exit code
    before the node or controller are started (the actions below are only returned
    on the success path).
    """
    pkg_share_folder = get_package_share_directory("pal_policy_deployer")
    task = LaunchConfiguration("task").perform(context)
    model = LaunchConfiguration("model").perform(context)
    imitation_task = LaunchConfiguration("imitation_task").perform(context)
    imitation_model = LaunchConfiguration("imitation_model").perform(context)
    imitation_frames = LaunchConfiguration("imitation_frames").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    activate_impedance_controller = LaunchConfiguration("activate_impedance_controller").perform(context)

    # Fail fast (non-zero exit) before launching anything if either model is absent.
    model_dir = resolve_policy_model(pkg_share_folder, task, model)
    imitation_model_dir = resolve_policy_model(pkg_share_folder, imitation_task, imitation_model)

    # The first policy in the list is the default-active one at startup; the node
    # switches to index 1 (the imitation policy) when it receives a SwitchPolicy goal.
    kang_mjlab_node = Node(
        package="pal_policy_inferer",
        executable="kang_mjlab_node",
        name="pal_policy_deployer",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "policies": ["loco", "imitation"],
            "loco.task": task,
            "loco.model_path": model_dir,
            "imitation.task": imitation_task,
            "imitation.model_path": imitation_model_dir,
            "imitation.frames": int(imitation_frames),
        }],
        remappings=[
            ("/subscriber_controller/desired_state", "/rl_impedance_controller/reference"),
        ],
    )

    if activate_impedance_controller == "True":
        extra_spawner_args = ["--unload-on-kill"]
    else:
        extra_spawner_args = ["--inactive"]
    rl_impedance_controller = GroupAction(
        [generate_load_controller_launch_description(
            controller_name="rl_impedance_controller",
            controller_params_file=os.path.join(
                pkg_share_folder,
                "config", f"{robot_type}.yaml"),
            extra_spawner_args=extra_spawner_args)
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

    policies_info = LogInfo(
        msg=(
            f"[pal_policy_deployer] Deploying 2 policies: [0] loco '{model}' (active by "
            f"default), [1] {imitation_task} '{imitation_model}'. Switch to the imitation "
            "policy with: ros2 action send_goal /pal_policy_deployer/switch_policy "
            "pal_policy_inferer/action/SwitchPolicy \"{policy_index: 1}\""
        )
    )

    return [policies_info, kang_mjlab_node, rl_impedance_controller, shutdown_on_node_exit]


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    launch_description.add_action(OpaqueFunction(function=_check_model_and_create_actions))


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
