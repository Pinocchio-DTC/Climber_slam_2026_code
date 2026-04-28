from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    nav_pose_yaml = PathJoinSubstitution([
        FindPackageShare("cod_behavior"),
        "launch",
        "cod_pose_tactical_front.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="front_patrol"
        ),
        Node(
            package="cod_behavior",
            executable="tree_1",
            name="cod_behavior",
            output="screen",
            parameters=[nav_pose_yaml],
            arguments=["--ros-args", "--log-level", "warn"]
        ),
        Node(
            package="cod_behavior",
            executable="test_tactical_serial",
            name="test_tactical_serial",
            output="screen",
            parameters=[{"scenario": scenario}],
            arguments=["--ros-args", "--log-level", "warn"]
        )
    ])
