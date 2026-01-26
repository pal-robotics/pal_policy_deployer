from dataclasses import dataclass

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch_pal.arg_utils import LaunchArgumentsBase

# Definition of opaque function
def _build_metadata_map(context, *args, **kwargs) -> list[SetLaunchConfiguration]:
    import onnx
    onnx_model_path = LaunchConfiguration("onnx_model_path").perform(context)
    model = onnx.load(onnx_model_path)

    graph = model.graph

    input_tensor = graph.input[0].type.tensor_type
    output_tensor = graph.output[0].type.tensor_type

    input_shape = tuple(dim.dim_value for dim in input_tensor.shape.dim)
    output_shape = tuple(dim.dim_value for dim in output_tensor.shape.dim)

    input_shape = input_shape[-1]
    output_shape = output_shape[-1]

    metadata_params = []
    for prop in model.metadata_props:
        metadata_params.append(SetLaunchConfiguration(name = prop.key, value = prop.value))

    metadata_params.append(SetLaunchConfiguration(name = "input_dimension", value = str(input_shape)))
    metadata_params.append(SetLaunchConfiguration(name = "output_dimension", value = str(output_shape)))
    return metadata_params 

# Path definitions
DEFAULT_MODEL_PATH = PathJoinSubstitution([
    FindPackageShare("pal_policy_deployer"),
    "models",
    "tiagopro",
    "reaching_tiagopro.onnx",
])

# Launch file 
@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )

    onnx_model_path: DeclareLaunchArgument = DeclareLaunchArgument(
        name="onnx_model_path",
        default_value=DEFAULT_MODEL_PATH,
        description="Absolute path to the ONNX model file.",
    )

    preprocessor_params: DeclareLaunchArgument = DeclareLaunchArgument(
        name="preprocessor_params",
        default_value="",
        description="Path to the preprocessor parameters file.",
    )

    postprocessor_params: DeclareLaunchArgument = DeclareLaunchArgument(
        name="postprocessor_params",
        default_value="",
        description="Path to the postprocessor parameters file.",
    )

def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
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
    
    launch_description.add_action(OpaqueFunction(function=_build_metadata_map))

    policy_nodes = GroupAction([
        Node(
            package="pal_policy_preprocessor",
            executable="pal_policy_preprocessor_node",
            name="pal_policy_preprocessor",
            output="screen",
            parameters=[
                PREPROCESSOR_PARAMS,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "input_dimension": ParameterValue(LaunchConfiguration("input_dimension"), value_type=int),
                    "output_dimension": ParameterValue(LaunchConfiguration("output_dimension"), value_type=int),
                    "joint_names": LaunchConfiguration("joint_names"),
                    "default_joint_pos": LaunchConfiguration("default_joint_pos"),
                }
            ],
        ),

        Node(
            package="pal_policy_inferer",
            executable="pal_policy_inferer_node",
            name="pal_policy_inferer",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "onnx_model_path": LaunchConfiguration("onnx_model_path"),
                "input_dimension": ParameterValue(LaunchConfiguration("input_dimension"), value_type=int),
                "output_dimension": ParameterValue(LaunchConfiguration("output_dimension"), value_type=int),
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
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "input_dimension": ParameterValue(LaunchConfiguration("input_dimension"), value_type=int),
                    "output_dimension": ParameterValue(LaunchConfiguration("output_dimension"), value_type=int),
                    "action_scale": LaunchConfiguration("action_scale"),
                    "default_joint_pos": LaunchConfiguration("default_joint_pos"),
                }
            ],
        ),
        Node(
            package="pal_policy_preprocessor",
            executable="interactive_marker_node",
            name="interactive_target_rviz",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])

    launch_description.add_action(policy_nodes)


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld

