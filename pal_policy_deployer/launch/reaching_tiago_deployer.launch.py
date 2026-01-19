from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    onnx_model_path = LaunchConfiguration("onnx_model_path")

    default_model_path = PathJoinSubstitution([
        FindPackageShare("pal_policy_deployer"),
        "models", "tiagopro", "exported", "reaching_tiagopro.onnx"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulation (Gazebo) clock if true, false for robot."
        ),

        DeclareLaunchArgument(
            "onnx_model_path",
            default_value=default_model_path,
            description="Absolute path to the ONNX model file."
        ),

        Node(
            package="pal_policy_preprocessor",
            executable="pal_policy_preprocessor_node",
            name="pal_policy_preprocessor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "onnx_model_path": onnx_model_path,
            }],
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
            parameters=[{
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
