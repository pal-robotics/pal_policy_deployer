from dataclasses import dataclass

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_pal.arg_utils import LaunchArgumentsBase

# TO DO: load model path from config file
DEFAULT_MODEL_PATH = PathJoinSubstitution([
    FindPackageShare("pal_policy_deployer"),
    "models",
    "tiagopro",
    "reaching_tiagopro.onnx",
])

PREPROCESSOR_PARAMS = PathJoinSubstitution([
    FindPackageShare("pal_policy_preprocessor"),
    "config",
    "tiago_pro",
    "params.yaml",
])

POSTPROCESSOR_PARAMS = PathJoinSubstitution([
    FindPackageShare("pal_policy_postprocessor"),
    "config",
    "tiago_pro",
    "params.yaml",
])


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock.",
    )

    onnx_model_path: DeclareLaunchArgument = DeclareLaunchArgument(
        name="onnx_model_path",
        default_value=DEFAULT_MODEL_PATH,
        description="Absolute path to the ONNX model file.",
    )


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    use_sim_time = LaunchConfiguration("use_sim_time")
    onnx_model_path = LaunchConfiguration("onnx_model_path")

    policy_nodes = GroupAction([
        Node(
            package="pal_policy_preprocessor",
            executable="pal_policy_preprocessor_node",
            name="pal_policy_preprocessor",
            output="screen",
            parameters=[
                PREPROCESSOR_PARAMS,
                {
                    "use_sim_time": use_sim_time,
                    "onnx_model_path": onnx_model_path,
                }
            ],
        ),

        Node(
            package="pal_policy_inferer",
            executable="pal_policy_inferer_node",
            name="pal_policy_inferer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "onnx_model_path": onnx_model_path,
            }],
        ),

        Node(
            package="pal_policy_postprocessor",
            executable="pal_policy_postprocessor_node",
            name="pal_policy_postprocessor",
            output="screen",
            parameters=[
                POSTPROCESSOR_PARAMS,
                {
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
    ])

    launch_description.add_action(policy_nodes)


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
