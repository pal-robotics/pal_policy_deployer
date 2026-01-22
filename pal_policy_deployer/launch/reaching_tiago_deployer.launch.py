from dataclasses import dataclass

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_pal.arg_utils import LaunchArgumentsBase



# Definition of opaque functions

def _build_metadata_map(onnx_model_path: str) -> dict:
    import onnxruntime as ort
    if not onnx_model_path:
        raise ValueError("onnx_model_path is empty")

    session_options = ort.SessionOptions()
    session = ort.InferenceSession(onnx_model_path, session_options)

    metadata = session.get_modelmeta()
    metadata_map = {}

    for key in metadata.custom_metadata_map.keys():
        value = metadata.custom_metadata_map[key]
        metadata_map[key] = value

    return metadata_map

def _derive_input_output_dimensions(onnx_model_path: str) -> tuple[int, int]:
    import onnxruntime as ort
    if not onnx_model_path:
        raise ValueError("onnx_model_path is empty")

    session_options = ort.SessionOptions()
    session = ort.InferenceSession(onnx_model_path, session_options)

    inputs = session.get_inputs()
    outputs = session.get_outputs()

    input_shape = inputs[0].shape 
    input_dimension = int(input_shape[-1])

    output_shape = outputs[0].shape
    output_dimension = 1
    for d in output_shape:
        output_dimension *= int(d)

    return input_dimension, output_dimension

# Path definitions
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


# Launch file 
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


def declare_actions(context, *args, **kwargs):
    use_sim_time = bool(LaunchConfiguration("use_sim_time").perform(context))
    onnx_model_path = LaunchConfiguration("onnx_model_path").perform(context)

    metadatamap = _build_metadata_map(onnx_model_path)
    in_dim, out_dim = _derive_input_output_dimensions(onnx_model_path)

    # Flatten metadata into parameters under a prefix
    metadata_params = {f"model_metadata.{k}": v for k, v in metadatamap.items()}

    # Scalar model IO parameters (native int)
    input_dimension = {"model_io.input_dimension": in_dim}
    output_dimension = {"model_io.output_dimension": out_dim}

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
                    **metadata_params,
                    **input_dimension,
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
                **input_dimension,
                **output_dimension,
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
                    **metadata_params,
                    **output_dimension,
                }
            ],
        ),
    ])

    return [policy_nodes]


def generate_launch_description():
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()
    launch_arguments.add_to_launch_description(ld)
    ld.add_action(OpaqueFunction(function=declare_actions))

    return ld




